#ifndef BF_SPI3_H
#define BF_SPI3_H

#include "gpio.h"
#include "spi.h"
#include "stm32u5xx_hal_spi.h"
#include <stdint.h>

// Reinitialize SPI3 with a specific word size (17/30/31-bit). All other
// settings mirror the CubeMX-generated defaults so timing stays consistent.
HAL_StatusTypeDef BF_SPI3_ReInit(uint32_t datasize);

// Convenience helper: pick the correct word size when you know the total
// bits in your payload. Examples: 60 -> 2x30-bit frames, 62 -> 2x31-bit,
// 34 -> 2x17-bit. Anything else is rejected.
HAL_StatusTypeDef BF_SPI3_ReInitForBits(uint16_t total_bits);

// Turn off SPI3 and its clock cleanly.
void BF_SPI3_DeInit(void);

// Transmit two frames (MSB first) using the current word size. We always send
// two frames because the beamformer packets are split into pairs (60, 62, 34).
// Caller must ensure the SPI is already configured for the matching datasize.
HAL_StatusTypeDef BF_SPI3_Send2Words(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const uint32_t words[2]);

#endif // BF_SPI3_H
