#ifndef BF_SPI_H
#define BF_SPI_H

#include "spi.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_spi.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

// Select the SPI instance used for beamformer transfers.
// Switch to &hspi2 here if the board routes beamformer SPI to SPI2.
#define BF_SPI_HANDLE (&hspi1)

HAL_StatusTypeDef BF_SPI_ReInitForBits(uint16_t total_bits);
HAL_StatusTypeDef BF_SPI_ReInitDataSize(uint32_t datasize);
void BF_SPI_DeInit(void);
HAL_StatusTypeDef BF_SPI_Send2Words(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                    const uint32_t words[2]);

#endif // BF_SPI_H
