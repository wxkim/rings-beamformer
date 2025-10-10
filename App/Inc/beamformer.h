#ifndef BEAMFORMER_H
#define BEAMFORMER_H

#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_def.h"
#include "stm32u5xx_hal_spi.h"
#include <stdint.h>

// #include "usb_device.h"

// add code later for initializing-deinitalizing spi protocol with custom word
// sizes for command
void SPI1_SetWordSize(uint32_t datasize);
extern SPI_HandleTypeDef hspi1;

// We declare explicit command codes matching a packer
typedef enum
{
    BF_CMD_REG_WR = 0x0, //normal register write
    BF_CMD_BROADCAST_WR = 0x2 //broadcast write
} bf_cmd_t;


//60-bit serial register frame
typedef struct{
    bf_cmd_t cmd; // command type
    uint16_t addr10;
    uint64_t data48;
} bf_register_frame_t;

// 62-bit broadcast frame - write to all IC's
typedef struct {
  uint16_t addr10;
  uint64_t data48;
} bf_broadcast_frame_t;


//34-bit fast-beam frame
typedef struct{
    uint8_t tdbs_addr_B;
    uint8_t tdbs_addr_A;
    uint16_t fbs_addr_B;
    uint16_t fbs_addr_A;
    uint8_t is_tx_bank; //0 is RX(1110), 1 = TX (1111)
} bf_fastbeam_frame_t;

// build 60-bit register frame from uint_64

void BF_Pack60(const bf_register_frame_t *f);

// split 60-bit into 4x15 word chunks
void BF_Pack60_to4x15(const bf_register_frame_t *f, uint16_t out[4]);

     // build 62-bit broadcast frame from uint_64
void BF_Pack62(const bf_broadcast_frame_t *f);

// split 34-bit into 4x15 word chunks
void BF_Pack34 (const bf_fastbeam_frame_t *f, uint8_t out[5]);


//Transmit into register

// Transmit into register

// send one 60-bit register frame
HAL_StatusTypeDef BF_Send60(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                            uint16_t cs_pin, const bf_register_frame_t *f);

// send one 62-bit broadcast frame
HAL_StatusTypeDef BF_Send62_Broadcast(SPI_HandleTypeDef *hspi,
                                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                      const bf_broadcast_frame_t *f);

// Send N-chained 60-bit register frames
HAL_StatusTypeDef BF_Send60_Chain(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  const bf_register_frame_t *f, uint16_t N);

// send one 34-bit fast-beam frame
HAL_StatusTypeDef BF_Send34_FastBeam(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const bf_fastbeam_frame_t *f);

#endif // BEAMFORMER_H
