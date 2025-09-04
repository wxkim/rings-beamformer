#ifndef BEAMFORMER_H
#define BEAMFORMER_H

#include "gpio.h"
#include "main.h"
#include "spi.h"
#include <stdint.h>
// #include "usb_device.h"

typedef enum {
    COMMAND_SERIAL_RDWR,
    COMMAND_SERIAL_RDWR_CHAIN,
    COMMAND_HYBRID_RDWR_CHAIN,
    COMMAND_BROADCAST_WR,
    COMMAND_RX_BEAMSTEERING,
    COMMAND_TX_BEAMSTEERING,
} beamformer_command_type;

typedef struct {
    uint8_t parity:1;
    uint8_t buffered_write:1; // 1 = buffered write, 0 = immediate write
    uint8_t reg_addr:7;
    uint64_t data:48;
    uint8_t TDBS_addr_A:6;
    uint8_t TDBS_addr_B:6;
    uint16_t FBS_addr_A:9;
    uint16_t FBS_addr_B:9;
    beamformer_command_type command_type;
} beamformer_command_cfg_t;

void pack_command(beamformer_command_cfg_t *config);

#endif // BEAMFORMER_H
