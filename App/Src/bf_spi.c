#include "bf_spi.h"

// Map total bit counts to the required datasize setting.
static uint32_t BF_SPI_DatasizeForBits(uint16_t total_bits) {
  switch (total_bits) {
  case 34:
    return SPI_DATASIZE_17BIT;
  case 60:
    return SPI_DATASIZE_30BIT;
  case 62:
    return SPI_DATASIZE_31BIT;
  default:
    return 0;
  }
}

HAL_StatusTypeDef BF_SPI_ReInitDataSize(uint32_t datasize) {
  if (datasize == 0) {
    return HAL_ERROR;
  }

  SPI_HandleTypeDef *hspi = BF_SPI_HANDLE;
   if (hspi == NULL || hspi->Instance == NULL) {
    return HAL_ERROR;
  }

  HAL_SPI_StateTypeDef st = HAL_SPI_GetState(hspi);
  if (st != HAL_SPI_STATE_READY && st != HAL_SPI_STATE_RESET) {
    return HAL_BUSY;
  }

  if (st != HAL_SPI_STATE_RESET) {
    if (HAL_SPI_DeInit(hspi) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  hspi->Init.DataSize = datasize;
  if (HAL_SPI_Init(hspi) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef BF_SPI_ReInitForBits(uint16_t total_bits) {
  uint32_t datasize = BF_SPI_DatasizeForBits(total_bits);
  if (datasize == 0) {
    return HAL_ERROR;
  }
  return BF_SPI_ReInitDataSize(datasize);
}

void BF_SPI_DeInit(void) {
  SPI_HandleTypeDef *hspi = BF_SPI_HANDLE;
  if (hspi != NULL && hspi->Instance != NULL) {
    HAL_SPI_DeInit(hspi);
  }
}

HAL_StatusTypeDef BF_SPI_Send2Words(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                    const uint32_t words[2]) {
  if (cs_port == NULL || words == NULL) {
    return HAL_ERROR;
  }

  SPI_HandleTypeDef *hspi = BF_SPI_HANDLE;
  if (hspi == NULL || hspi->Instance == NULL) {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hspi, (uint8_t *)words, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef BF_SPI_Tx2Words_NoCS(const uint32_t words[2]) {
  if (words == NULL) {
    return HAL_ERROR;
  }

  SPI_HandleTypeDef *hspi = BF_SPI_HANDLE;
  if (hspi == NULL || hspi->Instance == NULL) {
    return HAL_ERROR;
  }

  return HAL_SPI_Transmit(hspi, (uint8_t *)words, 2, HAL_MAX_DELAY);
}
