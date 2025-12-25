#include "beamformer.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_def.h" // Add this line to define HAL_StatusTypeDef
#include "stm32u5xx_hal_gpio.h"
#include "stm32u5xx_hal_spi.h"
#include "bf_spi3.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi3;

// 60-bit packer

uint64_t BF_Pack60(const bf_register_frame_t *f) {

  // Serial writes per Anokiwave spec: leading bits are always 00.
  uint8_t ctrl = 0b00;
  uint64_t addr = (uint64_t)(f->addr10 & 0x03FFu);           // 10 bits
  uint64_t data = (uint64_t)(f->data48 & 0xFFFFFFFFFFFFull); // 48 bits

  uint64_t word = 0;
  word |= ((uint64_t)ctrl << 58); // ctrl -> bits [59:58]
  word |= (addr << 48);           // addr -> bits [57:48]
  word |= data;                   // data -> bits [47:0]
  return word;
}

uint64_t BF_Pack62(const bf_broadcast_frame_t *f) {
  uint64_t addr = (uint64_t)(f->addr10 & 0x03FFu);           // 10 bits
  uint64_t data = (uint64_t)(f->data48 & 0xFFFFFFFFFFFFull); // 48 bits

  uint64_t word = 0;
  word |= (1ull << 61); // Leading 1 at bit 61
  word |= (addr << 51); // ADDR in bits [60:51]
  word |= (data << 3);  // DATA in bits [50:3]
  // lower 3 bits are 0
  return word;
}

void BF_Pack62_to2x31(const bf_broadcast_frame_t *f, uint32_t out[2]) {
  if (!out)
    return;

  uint64_t word = BF_Pack62(f);
  out[0] = (uint32_t)((word >> 31) & 0x7FFFFFFF); // bits 61-31
  out[1] = (uint32_t)(word & 0x7FFFFFFF);         // bits 30-0
}

void BF_Pack34(const bf_fastbeam_frame_t *f, uint8_t out[5]) {
  if (!out)
    return;

  uint64_t word = 0;

  uint8_t opcode = (f->is_tx_bank) ? 0b1111 : 0b1110; // TX or RX bank

  word |= ((uint64_t)opcode << 30); // safe cast, no overflow - opcode bits [33:30]
  word |= ((uint64_t)(f->tdbs_addr_B & 0x3F) << 24); // TDBS B bits [29:24]
  word |= ((uint64_t)(f->tdbs_addr_A & 0x3F) << 18); // TDBS A bits [23:18]
  word |= ((uint64_t)(f->fbs_addr_B & 0x1FF) << 9);  // FBS B bits [17:9]
  word |= ((uint64_t)(f->fbs_addr_A & 0x1FF) << 0);  // FBS A bits [8:0]

  // Align into MSB of 64 bit

  uint64_t aligned = word << 30; // shift left by 30 to align to MSB

  // Now into 5 bytes

  for (int i = 0; i < 5; i++) {
    out[i] = (uint8_t)((aligned >> (56 - i * 8)) &
                       0xFF); //  bit position 64-56 are put into out[0]...

    // uint8_t ctrl = ();
  }
}

void BF_Pack34_to2x17(const bf_fastbeam_frame_t *f, uint32_t out[2]) {
  if (!out)
    return;

  uint64_t word = 0;
  uint8_t opcode = (f->is_tx_bank) ? 0b1111 : 0b1110;

  word |= ((uint64_t)opcode << 30);
  word |= ((uint64_t)(f->tdbs_addr_B & 0x3F) << 24);
  word |= ((uint64_t)(f->tdbs_addr_A & 0x3F) << 18);
  word |= ((uint64_t)(f->fbs_addr_B & 0x1FF) << 9);
  word |= ((uint64_t)(f->fbs_addr_A & 0x1FF) << 0);

  out[0] = (uint32_t)((word >> 17) & 0x1FFFF); // bits 33-17
  out[1] = (uint32_t)(word & 0x1FFFF);         // bits 16-0
}

void BF_Pack60_to4x15(const bf_register_frame_t *f,
                      uint16_t out[4]) // MSB first structure
{
  if (!out)
    return;

  uint64_t word = BF_Pack60(f);             // check
  out[0] = (uint16_t)(word >> 45) & 0x7FFF; // bits 59-45
  out[1] = (uint16_t)(word >> 30) & 0x7FFF; // bits 44-30
  out[2] = (uint16_t)(word >> 15) & 0x7FFF; // bits 29-15
  out[3] = (uint16_t)(word) & 0x7FFF;       // bits 14-0
}

void BF_Pack60_to2x30(const bf_register_frame_t *f, uint32_t out[2]) {
  if (!out)
    return;

  uint64_t word = BF_Pack60(f);
  out[0] = (uint32_t)((word >> 30) & 0x3FFFFFFF); // bits 59-30
  out[1] = (uint32_t)(word & 0x3FFFFFFF);         // bits 29-0
}

// Transmit Helpers

// send one 60-bit register frame
HAL_StatusTypeDef BF_Send60(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                            uint16_t cs_pin, const bf_register_frame_t *f) {
  (void)hspi; // kept for API compatibility
  uint64_t word = BF_Pack60(f);
  return BF_SPI3_SendBits(cs_port, cs_pin, word, 60);
}

// Send N-chained 60-bit register frames
HAL_StatusTypeDef BF_Send60_Chain(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  const bf_register_frame_t *f, uint16_t N) {
  (void)hspi; // kept for API compatibility
  BF_SPI3_BitBangInit();

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); // CS low for whole chain

  for (int frame = 0; frame < N; frame++) {
    uint64_t word = BF_Pack60(&f[frame]);
    BF_SPI3_ShiftBits(word, 60);
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  return HAL_OK;
}

// send one 62-bit broadcast frame
HAL_StatusTypeDef BF_Send62_Broadcast(SPI_HandleTypeDef *hspi,
                                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                      const bf_broadcast_frame_t *f) {

  (void)hspi; // kept for API compatibility
  uint64_t word = BF_Pack62(f);
  return BF_SPI3_SendBits(cs_port, cs_pin, word, 62);
}

// send one 34-bit fast-beam frame
HAL_StatusTypeDef BF_Send34_FastBeam(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const bf_fastbeam_frame_t *f) {
  (void)hspi; // kept for API compatibility
  uint64_t word = 0;
  uint8_t opcode = (f->is_tx_bank) ? 0b1111 : 0b1110; // TX or RX bank

  word |= ((uint64_t)opcode << 30);
  word |= ((uint64_t)(f->tdbs_addr_B & 0x3F) << 24);
  word |= ((uint64_t)(f->tdbs_addr_A & 0x3F) << 18);
  word |= ((uint64_t)(f->fbs_addr_B & 0x1FF) << 9);
  word |= ((uint64_t)(f->fbs_addr_A & 0x1FF) << 0);

  // Send exactly 34 bits, MSB first.
  return BF_SPI3_SendBits(cs_port, cs_pin, word, 34);
}
