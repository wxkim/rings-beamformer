#include "Drivers/STM32U5xx_HAL_Driver/Inc/stm32u5xx_hal.h"
#include "stm32u5xx_hal_spi.h"
#include "stm32u5xx_hal_gpio.h"
#include "beamformer.h"
#include "stm32u5xx_hal_def.h" // Add this line to define HAL_StatusTypeDef


#include <stdint.h>

extern SPI_HandleTypeDef hspi1;


// SPI size setter function - call this before sending frames on SPI
void SPI1_SetWordSize(uint32_t datasize) {
    hspi1.Init.DataSize = datasize;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

//60-bit packer
//hspi1.init.DataSize = SPI_DATASIZE_15BIT;

uint64_t BF_Pack60(const bf_register_frame_t *f) {
  
  

    uint8_t ctrl = (f->cmd == BF_CMD_BROADCAST_WR) ? 0b10 : 0b00; //00 = reg write, 10 = broadcast write
    uint64_t addr = (uint64_t)(f->addr10 & 0x03FFu); //10 bits
    uint64_t data = (uint64_t)(f->data48 & 0xFFFFFFFFFFFFull); //48 bits

    uint64_t word = 0;
    word |= ((uint64_t)ctrl << 58); //ctrl -> bits [59:58]
    word |= (addr << 48);           //addr -> bits [57:48]
    word |= data;                   //data -> bits [47:0]
    return word;
}


uint64_t BF_Pack62(const bf_broadcast_frame_t *f)
{
    uint8_t ctrl = 0b10; //broadcast write
    uint64_t addr = (uint64_t)(f->addr10 & 0x03FFu); //10 bits
    uint64_t data = (uint64_t)(f->data48 & 0xFFFFFFFFFFFFull); //48 bits

    uint64_t word = 0;
    word |= (1ull << 61); // Leading 1 at bit 61
    word |= (addr << 51); // ADDR in bits [60:51]
    word |= (data << 3); // DATA in bits [50:3]
    //lower 3 bits are 0
    return word;



}


void BF_Pack34 (const bf_fastbeam_frame_t *f, uint8_t out[5])
{
    if(!out) return;

    uint64_t word = 0;

    uint8_t opcode = (f->is_tx_bank) ? 0b1111 : 0b1110; //TX or RX bank

    word |= ((uint64_t)opcode << 30);  // safe cast, no overflow - opcode bits [33:30]
    word |= ((uint64_t)(f->tdbs_addr_B & 0x3F) << 24); // TDBS B bits [29:24]
    word |= ((uint64_t)(f->tdbs_addr_A & 0x3F) << 18); // TDBS A bits [23:18]
    word |= ((uint64_t)(f->fbs_addr_B & 0x3FF) << 8); // FBS B bits [17:8]
    word |= ((uint64_t)(f->fbs_addr_A & 0xFF) << 0); // FBS A bits [7:0]

    //Align into MSB of 64 bit

    uint64_t aligned = word << 30; //shift left by 30 to align to MSB

    //Now into 5 bytes

    for (int i = 0; i < 5; i++)
    {
        out[i] = (uint8_t)((aligned >> (56 - i * 8)) & 0xFF); //  bit position 64-56 are put into out[0]...

        //uint8_t ctrl = ();
    }
   

}

void BF_Pack60_to4x15(const bf_register_frame_t *f, uint16_t out[4]) //MSB first structure
{
    if (!out)
        return;

    uint64_t word = BF_Pack60(f); //check
    out[0] = (uint16_t)(word >> 45) & 0x7FFF; //bits 59-45
    out[1] = (uint16_t)(word >> 30) & 0x7FFF; //bits 44-30
    out[2] = (uint16_t)(word >> 15) & 0x7FFF; //bits 29-15
    out[3] = (uint16_t)(word) & 0x7FFF; //bits 14-0
}






//Transmit Helpers

//send one 60-bit register frame
HAL_StatusTypeDef BF_Send60(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const bf_register_frame_t *f)
{
  SPI1_SetWordSize(SPI_DATASIZE_15BIT);
 //reinitialize spi for 15 bit word size
      
    uint16_t chunks[4] = {0};
    BF_Pack60_to4x15(f, chunks);


    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); //CS low

    for(int i = 0; i < 4; i++)
    {
        HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, (uint8_t *)&chunks[i], 1, HAL_MAX_DELAY); //send 15 bits based on SPI configured to 15 bits "1"
        if (status != HAL_OK)
        {
            HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); //CS high
            return status; //transmit error
        }
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); //CS high
    return HAL_OK;

    //deinit spi or reinit to 8 bit later if needed
}

//Send N-chained 60-bit register frames
HAL_StatusTypeDef BF_Send60_Chain(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  const bf_register_frame_t *f, uint16_t N) {
   SPI1_SetWordSize(SPI_DATASIZE_15BIT);
//reinitialize spi for 15 bit word size
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); //CS low

for(int frame = 0; frame < N; frame++)
{
    uint16_t chunks[4] = {0};
    BF_Pack60_to4x15(&f[frame], chunks);

    for(int i = 0; i < 4; i++)
    {
        HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, (uint8_t *)&chunks[i], 1, HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); //CS high
            return status; //transmit error
        }
    }
}
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); //CS high
return HAL_OK;
}


//send one 62-bit broadcast frame
HAL_StatusTypeDef BF_Send62_Broadcast(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const bf_broadcast_frame_t *f)
{

  SPI1_SetWordSize(SPI_DATASIZE_8BIT);
 // ensure 8-bit mode for 5-byte transmit

    uint64_t word = BF_Pack62(f);
    uint8_t bytes[8] = {0};

    //align to MSB
    for(int i = 0; i < 8; i++)
    {
        bytes[i] = (uint8_t)((word >> (56 - i * 8)) & 0xFF);
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, bytes, 8, HAL_MAX_DELAY); //sends 64 instead of 62 - figure out 62
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
return status;

}

//send one 34-bit fast-beam frame
HAL_StatusTypeDef BF_Send34_FastBeam(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const bf_fastbeam_frame_t *f)
{
   SPI1_SetWordSize(SPI_DATASIZE_8BIT);
 // ensure 8-bit mode for 5-byte transmit

    uint8_t bytes[5] = {0}; //5 bytes = 40 bits
    BF_Pack34(f, bytes);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); //CS low
    if (HAL_SPI_Transmit(hspi, bytes, 5, HAL_MAX_DELAY) != HAL_OK) // send 40 bits
      {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return HAL_ERROR;
}
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
return HAL_OK;
}








