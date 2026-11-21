/**
 * @file button.c
 * @brief Hardware button (EXT1) implementation with press detection.
 * @details Implements GPIO interrupt-based button handling with timing-based
 *          press type detection (short vs long). Handles platform differences
 *          between RP2040 and RP2350 pull resistor configurations.
 *          
 *          Timing thresholds:
 *          - Short press: <550ms
 *          - Long press: ≥550ms
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pirate.h"
#include "system_config.h"
#include "pirate/button.h"
#include "hardware/structs/iobank0.h"

#define BP_BUTTON_SHORT_PRESS_MS 550

// volatile because these are modified in IRQ handler
static volatile bool button_pressed = false;
static volatile absolute_time_t press_start_time;
static volatile enum button_codes button_code = BP_BUTT_NO_PRESS;

// poll the value of button button_id
bool button_get(uint8_t button_id) {
    // pull-down on RP2040, active high
    #if RPI_PLATFORM == RP2040    
        return gpio_get(EXT1);
    //pull-up on RP2350 because of E9 silicon bug, active low
    #elif RPI_PLATFORM == RP2350  
        return !gpio_get(EXT1);
    #else
        #error "Platform not specified in button.c"
    #endif
}
// check button press type
enum button_codes button_check_press(uint8_t button_id) {
    if (button_pressed) {
        enum button_codes press_code = button_code;
        button_code = BP_BUTT_NO_PRESS;
        button_pressed = false;
        return press_code;
    }
    return BP_BUTT_NO_PRESS;
}
// The button is active high on RP2040 (pull-down) and active low on RP2350 (pull-up,
// E9 erratum), so which edge is the press depends on the platform.
#if RPI_PLATFORM == RP2040
    #define BUTTON_EDGE_PRESS GPIO_IRQ_EDGE_RISE
    #define BUTTON_EDGE_RELEASE GPIO_IRQ_EDGE_FALL
#elif RPI_PLATFORM == RP2350
    #define BUTTON_EDGE_PRESS GPIO_IRQ_EDGE_FALL
    #define BUTTON_EDGE_RELEASE GPIO_IRQ_EDGE_RISE
#endif

// true between a press edge and its release edge
static volatile bool press_in_progress = false;

// example irq callback handler, copy for your own uses
void button_irq_callback(uint gpio, uint32_t events) {
    if (events & BUTTON_EDGE_PRESS) {
        press_start_time = get_absolute_time();
        press_in_progress = true;
    }

    // Only a release that follows a press counts. A release seen first (button
    // held through boot or while the IRQ was disabled) has no start time.
    if ((events & BUTTON_EDGE_RELEASE) && press_in_progress) {
        press_in_progress = false;
        absolute_time_t press_end_time = get_absolute_time();
        int64_t duration_ms = absolute_time_diff_us(press_start_time, press_end_time) / 1000;

        if (duration_ms >= BP_BUTTON_SHORT_PRESS_MS) {
            button_code = BP_BUTT_LONG_PRESS;
        } else {
            button_code = BP_BUTT_SHORT_PRESS;
        }

        button_pressed = true;
    }

    gpio_acknowledge_irq(gpio, events);
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}
// enable the irq for button button_id
void button_irq_enable(uint8_t button_id, gpio_irq_callback_t callback) {
    button_pressed = false;
    press_in_progress = false; // a button already held is ignored until its next press
    gpio_set_irq_enabled_with_callback(EXT1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, callback);
}
// disable the irq for button button_id
void button_irq_disable(uint8_t button_id) {
    gpio_set_irq_enabled(EXT1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    button_pressed = false;
}
// initialize all buttons
void button_init(void) {
    gpio_set_function(EXT1, GPIO_FUNC_SIO);
    gpio_set_dir(EXT1, GPIO_IN);
    #if (RPI_PLATFORM == RP2040)
        gpio_pull_down(EXT1);
    #elif (RPI_PLATFORM == RP2350)
        gpio_pull_up(EXT1);
    #else
        #error "Platform not speficied in button.c"
    #endif
}
