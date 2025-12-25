#ifndef BF_SPI3_H
#define BF_SPI3_H

#include "gpio.h"
#include "main.h"
#include "stm32u5xx_hal.h"
#include <stdint.h>

/**
 * SPI3 on STM32U575 only supports 8/16-bit frames. To hit the beamformer’s
 * exact 34/60/62-bit lengths, we bit-bang on the SPI3 pins (SCK/MOSI) with
 * manual CS. These helpers configure the pins for GPIO and shift MSB-first.
 */

// Prepare SPI3 pins for bit-bang mode (disables the SPI3 peripheral and sets
// SCK/MOSI as push-pull outputs). Safe to call repeatedly.
void BF_SPI3_BitBangInit(void);

// Shift bits with CS already controlled by the caller. SCK toggles for every
// bit; MOSI is driven MSB-first.
void BF_SPI3_ShiftBits(uint64_t word, uint8_t bit_count);

// Shift an arbitrary bit-count (<=64) MSB-first on the MOSI line while
// manually clocking SCK. CS is held low for the whole transfer. For broadcast
// use, ensure the MOSI line is wired to the ASIC PDI and SDI is held low.
HAL_StatusTypeDef BF_SPI3_SendBits(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                   uint64_t word, uint8_t bit_count);

#endif // BF_SPI3_H
