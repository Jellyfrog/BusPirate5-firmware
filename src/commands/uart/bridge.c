#include <stdio.h>
#include "pico/stdlib.h"
#include <stdint.h>
#include "hardware/uart.h"
#include "pirate.h"
#include "pirate/bio.h"
#include "system_config.h"
#include "command_struct.h"
#include "ui/ui_term.h"
#include "ui/ui_help.h"
#include "lib/bp_args/bp_cmd.h"
#include "bytecode.h"
#include "mode/hwuart.h"
#include "pirate/button.h"
#include "usb_rx.h"
#include "usb_tx.h"
#include "lib/bp_args/bp_cmd.h"
#include "ui/ui_toolbar.h"
#include "tusb.h"
#include "binmode/binmodes.h"

static const char* const usage[] = { "bridge\t[-h(elp)] [-t(oolbar)] [-b(aud follow)] [-r(eset pin) <IO>] [-g (boot pin) <IO>]",
                                     "Transparent UART bridge:%s bridge",
                                     "Follow host baud rate (esptool):%s bridge -b",
                                     "esptool auto-reset, EN on IO0, GPIO0 on IO1:%s bridge -b -r 0 -g 1",
                                     "Exit:%s press Bus Pirate button, send a serial BREAK, or send a BPIO2 request" };

static const bp_command_opt_t bridge_opts[] = {
    { "toolbar", 't', BP_ARG_NONE, NULL, T_HELP_UART_BRIDGE_TOOLBAR },
    { "baud", 'b', BP_ARG_NONE, NULL, T_HELP_UART_BRIDGE_BAUD },
    { "reset", 'r', BP_ARG_REQUIRED, "IO", T_HELP_UART_BRIDGE_RESET_PIN },
    { "boot", 'g', BP_ARG_REQUIRED, "IO", T_HELP_UART_BRIDGE_BOOT_PIN },
    { 0 }
};

const bp_command_def_t uart_bridge_def = {
    .name = "bridge",
    .description = T_HELP_UART_BRIDGE,
    .actions = NULL,
    .action_count = 0,
    .opts = bridge_opts,
    .usage = usage,
    .usage_count = count_of(usage),
};

#define BRIDGE_NO_PIN 0xFF

// Read an optional IO pin argument. Returns false (with an error printed) if the pin
// is out of range or already in use.
static bool bridge_get_pin(char flag, uint8_t* pin) {
    uint32_t value;
    *pin = BRIDGE_NO_PIN;
    if (!bp_cmd_get_uint32(&uart_bridge_def, flag, &value)) {
        return true; // not given
    }
    if (value >= count_of(bio2bufiopin)) {
        printf("%sError:%s pin IO%d is invalid\r\n", ui_term_color_error(), ui_term_color_reset(), value);
        return false;
    }
    if (system_config.pin_labels[value + 1] != 0) {
        printf("%sError:%s IO%d is in use by %s\r\n",
               ui_term_color_error(),
               ui_term_color_reset(),
               value,
               system_config.pin_labels[value + 1]);
        return false;
    }
    *pin = (uint8_t)value;
    return true;
}

static void bridge_pin_put(uint8_t pin, bool value) {
    if (pin != BRIDGE_NO_PIN) {
        bio_put(pin, value);
    }
}

// The classic ESP32/ESP8266 auto-reset circuit (two cross-wired transistors) driven
// from the host's DTR and RTS lines. Both lines asserted, or both released, leaves
// EN and GPIO0 high, so opening or closing the port does not reset the target.
static void bridge_autoreset_update(uint8_t reset_pin, uint8_t boot_pin) {
    uint8_t line_state = tud_cdc_n_get_line_state(0);
    bool dtr = line_state & 0x01;
    bool rts = line_state & 0x02;
    bridge_pin_put(reset_pin, !(rts && !dtr)); // EN low while RTS alone is asserted
    bridge_pin_put(boot_pin, !(dtr && !rts));  // GPIO0 low while DTR alone is asserted
}

void uart_bridge_handler(struct command_result* res) {
    if (bp_cmd_help_check(&uart_bridge_def, res->help_flag)) {
        return;
    }
    if (!ui_help_check_vout_vref()) {
        return;
    }

    uint8_t reset_pin, boot_pin;
    if (!bridge_get_pin('r', &reset_pin) || !bridge_get_pin('g', &boot_pin)) {
        res->error = true;
        return;
    }
    if (reset_pin != BRIDGE_NO_PIN && reset_pin == boot_pin) {
        printf("%sError:%s reset and boot pins must differ\r\n", ui_term_color_error(), ui_term_color_reset());
        res->error = true;
        return;
    }
    // claim the pins so they show on the display and nothing else takes them
    if (reset_pin != BRIDGE_NO_PIN) {
        bio_put(reset_pin, 1);
        bio_output(reset_pin);
        system_bio_update_purpose_and_label(true, reset_pin, BP_PIN_IO, "EN");
    }
    if (boot_pin != BRIDGE_NO_PIN) {
        bio_put(boot_pin, 1);
        bio_output(boot_pin);
        system_bio_update_purpose_and_label(true, boot_pin, BP_PIN_IO, "BOOT");
    }
    bool autoreset = (reset_pin != BRIDGE_NO_PIN) || (boot_pin != BRIDGE_NO_PIN);

    bool toolbar_paused = false;
    bool pause_toolbar = !bp_cmd_find_flag(&uart_bridge_def, 't');
    if (pause_toolbar) {
        toolbar_pause_updates();
        toolbar_paused = true;
    }

    printf("%s%s%s\r\n", ui_term_color_notice(), GET_T(T_HELP_UART_BRIDGE_EXIT), ui_term_color_reset());
    // IO7 is only the UART's RTS line with flow control on; otherwise it is a free pin
    // the user may be driving, so leave it alone.
    bool flow_control = hwuart_flow_control_enabled();
    if (flow_control) {
        bio_put(M_UART_RTS, 0);
    }
    usb_cdc_terminal_break_take(); // ignore a BREAK that arrived before the bridge started

    // With -b the target UART follows the baud rate the host sets on the terminal port,
    // like a USB serial adapter. esptool needs this to switch to a faster rate.
    bool follow_baud = bp_cmd_find_flag(&uart_bridge_def, 'b');
    bool baud_changed = false;
    uint32_t baud_seq = usb_cdc_terminal_baud_seq(); // only changes made from now on

    while (true) {
        if (autoreset) {
            bridge_autoreset_update(reset_pin, boot_pin);
        }
        if (follow_baud && baud_seq != usb_cdc_terminal_baud_seq()) {
            baud_seq = usb_cdc_terminal_baud_seq();
            uint32_t baud = usb_cdc_terminal_baud();
            if (baud) {
                uart_tx_wait_blocking(M_UART_PORT); // finish sending at the old rate
                hwuart_set_speed_temporary(baud);
                baud_changed = true;
            }
        }
        char c;
        if (rx_fifo_try_get(&c)) {
            uart_putc_raw(M_UART_PORT, c);
        }
        if (uart_is_readable(M_UART_PORT)) {
            c = uart_getc(M_UART_PORT);
            tx_fifo_put(&c);
        }
        // exit when button pressed.
        if (button_get(0)) {
            break;
        }
        // exit when the host sends a serial BREAK on the terminal port
        if (usb_cdc_terminal_break_take()) {
            break;
        }
        // The bridge blocks the core0 loop, so BPIO2 requests are not serviced while it
        // runs. Exit when one arrives; it is left unread for the BPIO2 handler.
        if (system_config.binmode_select == BINMODE_USE_DIRTYPROTO && tud_cdc_n_available(1)) {
            break;
        }
    }
    if (flow_control) {
        bio_put(M_UART_RTS, 1);
    }

    // release the auto-reset pins; the target's own pull-ups take over
    if (reset_pin != BRIDGE_NO_PIN) {
        bio_input(reset_pin);
        system_bio_update_purpose_and_label(false, reset_pin, BP_PIN_IO, 0);
    }
    if (boot_pin != BRIDGE_NO_PIN) {
        bio_input(boot_pin);
        system_bio_update_purpose_and_label(false, boot_pin, BP_PIN_IO, 0);
    }

    if (baud_changed) {
        uart_tx_wait_blocking(M_UART_PORT);
        hwuart_restore_speed();
    }

    if (toolbar_paused) {
        toolbar_resume_updates();
    }
}
