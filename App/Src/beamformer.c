#include "beamformer.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_def.h" // Add this line to define HAL_StatusTypeDef
#include "stm32u5xx_hal_gpio.h"
#include "stm32u5xx_hal_spi.h"
#include "bf_spi.h"

#include <stdint.h>

// 60-bit packer

uint64_t BF_Pack60(const bf_register_frame_t *f) {
  uint64_t addr = (uint64_t)(f->addr10 & 0x03FFu);           // 10 bits
  uint64_t data = (uint64_t)(f->data48 & 0xFFFFFFFFFFFFull); // 48 bits
  return (addr << 48) | data;
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
  out[0] = (uint32_t)(((word >> 31) & 0x7FFFFFFF) << 1); // bits 61-31 left aligned
  out[1] = (uint32_t)(((word >> 0) & 0x7FFFFFFF) << 1);  // bits 30-0 left aligned
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

  out[0] = (uint32_t)(((word >> 17) & 0x1FFFF) << 15); // bits 33-17 left aligned
  out[1] = (uint32_t)(((word >> 0) & 0x1FFFF) << 15);  // bits 16-0 left aligned
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
  out[0] = (uint32_t)(((word >> 30) & 0x3FFFFFFF) << 2); // bits 59-30 left aligned
  out[1] = (uint32_t)(((word >> 0) & 0x3FFFFFFF) << 2);  // bits 29-0 left aligned
}

// Transmit Helpers

// send one 60-bit register frame
HAL_StatusTypeDef BF_Send60(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                            const bf_register_frame_t *f) {
  if (BF_SPI_ReInitForBits(60) != HAL_OK) {
    return HAL_ERROR;
  }

  uint32_t chunks[2] = {0};
  BF_Pack60_to2x30(f, chunks);
  return BF_SPI_Send2Words(cs_port, cs_pin, chunks);
}

// Send N-chained 60-bit register frames
HAL_StatusTypeDef BF_Send60_Chain(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  const bf_register_frame_t *f, uint16_t N) {
  if (cs_port == NULL) {
    return HAL_ERROR;
  }
  if (f == NULL || N == 0) {
    return HAL_ERROR;
  }

  if (BF_SPI_ReInitForBits(60) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); // CS low

  for (int frame = 0; frame < N; frame++) {
    uint32_t chunks[2] = {0};
    BF_Pack60_to2x30(&f[frame], chunks);

    status = BF_SPI_Tx2Words_NoCS(chunks);
    if (status != HAL_OK) {
      break;
    }
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  return status;
}

// send one 62-bit broadcast frame
HAL_StatusTypeDef BF_Send62_Broadcast(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                      const bf_broadcast_frame_t *f) {

  if (BF_SPI_ReInitForBits(62) != HAL_OK) {
    return HAL_ERROR;
  }

  uint32_t chunks[2] = {0};
  BF_Pack62_to2x31(f, chunks);
  return BF_SPI_Send2Words(cs_port, cs_pin, chunks);
}

// send one 34-bit fast-beam frame
HAL_StatusTypeDef BF_Send34_FastBeam(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const bf_fastbeam_frame_t *f) {
  if (BF_SPI_ReInitForBits(34) != HAL_OK) {
    return HAL_ERROR;
  }

  uint32_t chunks[2] = {0};
  BF_Pack34_to2x17(f, chunks);
  return BF_SPI_Send2Words(cs_port, cs_pin, chunks);
}
