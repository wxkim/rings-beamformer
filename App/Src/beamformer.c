#include "beamformer.h"
#include <stdint.h>

void pack_command(beamformer_command_cfg_t *config) {
    switch(config->command_type){
        case COMMAND_SERIAL_RDWR:
            return /* pack command serial rdwr here */
    }
}

static void pack_command_serial_rdwr(beamformer_command_cfg_t * config) {
    uint64_t command = 0;

    command |= (config->parity << 58) | (config->reg_addr << 48);
    command |= (config->buffered_write << 55) | (config->data);
}