#ifndef BEAMFORMER_H
#define BEAMFORMER_H

#include "gpio.h"
#include "main.h"
#include "spi.h"
// #include "usb_device.h"

typedef struct {

} beamformer_command_t;

typedef enum {
    COMMAND_SERIAL_RDWR,
    COMMAND_SERIAL_RDWR_CHAIN,
    COMMAND_HYBRID_RDWR_CHAIN,
    COMMAND_BROADCAST_WR,
    COMMAND_RX_BEAMSTEERING,
    COMMAND_TX_BEAMSTEERING,
} beamformer_command_type;

#endif // BEAMFORMER_H
