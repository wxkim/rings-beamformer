#include "bf_spi3.h"

// SPI3 pin map from CubeMX:
// SCK  = PB3 (AF6)
// MOSI = PB5 (AF6) — routed to beamformer SDI/PDI (pins OR’ed on ASIC)
// NSS  = PA4  (unused here; CS is manual via passed-in GPIO)

// Small delay helper to keep SCK high/low long enough for the ASIC.
static inline void BF_SPI3_BitDelay(void) { __NOP(); }

static void BF_SPI3_ConfigPinsGPIO(void) {
  // Disable SPI3 peripheral so pins can be driven as GPIO.
  __HAL_RCC_SPI3_CLK_DISABLE();

  GPIO_InitTypeDef init = {0};
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // SCK (PB3)
  init.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &init);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  // MOSI/PDI (PB5)
  init.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &init);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void BF_SPI3_BitBangInit(void) { BF_SPI3_ConfigPinsGPIO(); }

static void BF_SPI3_ClockBit(GPIO_PinState mosi_state) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, mosi_state);
  // CPOL=0, CPHA=0: data valid on rising edge, latch after high phase.
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  BF_SPI3_BitDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  BF_SPI3_BitDelay();
}

void BF_SPI3_ShiftBits(uint64_t word, uint8_t bit_count) {
  if (bit_count == 0 || bit_count > 64) {
    return;
  }

  // Shift MSB-first.
  for (int i = bit_count - 1; i >= 0; --i) {
    GPIO_PinState bit = (word & (1ULL << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    BF_SPI3_ClockBit(bit);
  }
}

HAL_StatusTypeDef BF_SPI3_SendBits(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                   uint64_t word, uint8_t bit_count) {
  if (bit_count == 0 || bit_count > 64) {
    return HAL_ERROR;
  }

  BF_SPI3_BitBangInit();

  // Assert CS for the entire frame.
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  BF_SPI3_ShiftBits(word, bit_count);

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  return HAL_OK;
}
