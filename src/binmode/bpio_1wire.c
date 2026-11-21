#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pirate.h"
#include "bytecode.h"
#include "command_struct.h"
#include "bpio_reader.h"
#include "bpio_1wire.h"
#include "bpio_transactions.h"
#include "mode/hw1wire.h"
#include "pirate/hw1wire_pio.h"

#define ONEWIRE_SEARCH_ROM 0xF0
#define ONEWIRE_ROM_LEN 8

// Enumerate every device on the bus with the Search ROM algorithm (Maxim AN187).
// Byte-level transactions cannot do this: each ROM bit needs two reads and a write.
// Returns up to bytes_read / 8 ROM codes, 8 bytes each, in the order found.
// No devices is not an error, the response is just empty.
static uint32_t bpio_hw1wire_search(struct bpio_data_request_t *request, uint8_t *data_read) {
    uint32_t max_devices = request->bytes_read / ONEWIRE_ROM_LEN;
    if(max_devices == 0 || (request->bytes_read % ONEWIRE_ROM_LEN)) {
        request->error = "1-Wire search: bytes_read must be a non-zero multiple of 8";
        return true;
    }

    struct owobj search_owobj;
    uint32_t found = 0;
    int ret = OWFirst(&search_owobj);
    while(ret && found < max_devices) {
        memcpy(&data_read[found * ONEWIRE_ROM_LEN], search_owobj.ROM_NO, ONEWIRE_ROM_LEN);
        found++;
        ret = OWNext(&search_owobj);
    }
    if(request->debug) printf("[1WIRE] Search found %d device(s)\r\n", found);

    request->bytes_read_set = true;
    request->bytes_read_actual = found * ONEWIRE_ROM_LEN;
    return false;
}

uint32_t bpio_hw1wire_transaction(struct bpio_data_request_t *request, flatbuffers_uint8_vec_t data_write, uint8_t *data_read) {
    if(request->debug) printf("[1WIRE] Performing transaction\r\n");

    // start_alt with a lone Search ROM command runs the whole search on the device.
    // Sent as a plain byte it is useless: the host cannot answer the per-bit triplets.
    if(request->start_alt && request->bytes_write == 1 &&
       flatbuffers_uint8_vec_at(data_write, 0) == ONEWIRE_SEARCH_ROM) {
        return bpio_hw1wire_search(request, data_read);
    }

    if(request->start_main||request->start_alt) {
        if(request->debug) printf("[1WIRE] RESET\r\n");
        uint8_t device_detect = onewire_reset();
        if(!device_detect) {
            if(request->debug) printf("[1WIRE] No device detected\r\n");
            return true; 
        } else {
            if(request->debug) printf("[1WIRE] Device detected\r\n");
        }
    }

    if(request->bytes_write > 0) {
        if(request->debug) printf("[1WIRE] Writing %d bytes\r\n", request->bytes_write);
        for(uint32_t i = 0; i < request->bytes_write; i++) {
            onewire_tx_byte(flatbuffers_uint8_vec_at(data_write, i));
        }
        onewire_wait_for_idle(); // wait for the bus to be idle after writing
    }

    if(request->bytes_read > 0) {
        if(request->debug) printf("[1WIRE] Reading %d bytes\r\n", request->bytes_read);
        // read data
        for(uint32_t i = 0; i < request->bytes_read; i++) {
            data_read[i] = (uint8_t)onewire_rx_byte();
            onewire_wait_for_idle(); //temp test
        } 
    }

    return false;
}