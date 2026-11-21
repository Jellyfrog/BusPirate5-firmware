#pragma once

#include "hardware/pio.h"

struct _pio_config {
    PIO pio;
    uint sm;
    uint offset;
    const struct pio_program* program; // NULL while nothing is loaded
};

/*
 * PIO teardown goes through these helpers, never pio_remove_program() directly.
 * pio_remove_program() only marks the
 * instruction memory free: the state machine keeps running, and executes whatever
 * the next mode loads at those addresses, driving that program's pins. The SDK
 * call cannot stop it itself because it is not told which state machine(s) ran
 * the program.
 */

// Stop a state machine and free the program it ran. Leaves the SM claimed, if it
// was; the caller unclaims it.
static inline void pio_sm_release(PIO pio, uint sm, const pio_program_t* program, uint offset) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_remove_program(pio, program, offset);
}

// pio_sm_release() for a struct _pio_config. program == NULL means nothing is
// loaded, so a second release, or one before setup, does nothing.
static inline void pio_config_release(struct _pio_config* config) {
    if (config->program == NULL) {
        return;
    }
    pio_sm_release(config->pio, config->sm, config->program, config->offset);
    config->program = NULL;
}

// Free a program that no state machine was started on (setup error paths).
static inline void pio_program_unload(PIO pio, const pio_program_t* program, uint offset) {
    pio_remove_program(pio, program, offset);
}

static inline bool pio_sm_check_idle (PIO pio, int sm) {
    // Correct way to detect the state machine is idle because of an empty FIFO:
    // 1. Clear the (latching) state machine stall bit
    // 2. Check if the state machine TX fifo is empty
    // 3. THEN check if the state machine stall bit is set
    uint32_t SM_STALL_MASK = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    pio->fdebug = SM_STALL_MASK; // writing a 1 clears the bit

    // NOTE: the order of these operations *DOES* matter!
    bool result =
        pio_sm_is_tx_fifo_empty(pio, sm) &&
        (pio->fdebug & SM_STALL_MASK);
    return result;
}

static inline bool pio_sm_wait_idle (PIO pio, int sm, uint32_t timeout) {
    // Wait for the state machine to become idle
    while (!pio_sm_check_idle(pio, sm)) {
        timeout--;
        if (timeout == 0) {
            return false;
        }
    }
    return true;
}