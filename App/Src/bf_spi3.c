#include "bf_spi3.h"
#include "stm32u5xx_hal_spi_ex.h"

extern SPI_HandleTypeDef hspi3;

// Lightweight helpers to keep the SPI3 configuration in one place. The beam
// packets are always 34, 60, or 62 bits, so we only ever need 17/30/31-bit
// word sizes on SPI3.
static int BF_SPI3_WordsizeSupported(uint32_t datasize) {
  return (datasize == SPI_DATASIZE_17BIT || datasize == SPI_DATASIZE_30BIT ||
          datasize == SPI_DATASIZE_31BIT);
}

static uint32_t BF_SPI3_DatasizeFromBits(uint16_t total_bits) {
  // Map packet length to the exact SPI word size we must clock out.
  switch (total_bits) {
  case 60:
    return SPI_DATASIZE_30BIT;
  case 62:
    return SPI_DATASIZE_31BIT;
  case 34:
    return SPI_DATASIZE_17BIT;
  default:
    return 0;
  }
}

static void BF_SPI3_FillBaseConfig(uint32_t datasize) {
  // Match the CubeMX defaults except for the data size.
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = datasize;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
}

HAL_StatusTypeDef BF_SPI3_ReInit(uint32_t datasize) {
  if (!BF_SPI3_WordsizeSupported(datasize)) {
    return HAL_ERROR;
  }

  // Reset the peripheral before applying the new word size.
  HAL_SPI_DeInit(&hspi3);
  BF_SPI3_FillBaseConfig(datasize);

  SPI_AutonomousModeConfTypeDef auto_cfg = {0};
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    return HAL_ERROR;
  }

  auto_cfg.TriggerState = SPI_AUTO_MODE_DISABLE;
  auto_cfg.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
  auto_cfg.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &auto_cfg) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BF_SPI3_ReInitForBits(uint16_t total_bits) {
  uint32_t datasize = BF_SPI3_DatasizeFromBits(total_bits);
  if (datasize == 0) {
    return HAL_ERROR;
  }
  return BF_SPI3_ReInit(datasize);
}

void BF_SPI3_DeInit(void) {
  HAL_SPI_DeInit(&hspi3);
  __HAL_RCC_SPI3_CLK_DISABLE();
}

HAL_StatusTypeDef BF_SPI3_Send2Words(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const uint32_t words[2]) {
  // Only drive the bus when we have valid data and a supported word size.
  if (words == NULL || !BF_SPI3_WordsizeSupported(hspi3.Init.DataSize)) {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(&hspi3, (uint8_t *)words, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  return status;
}
