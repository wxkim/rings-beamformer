#include "beamformer.h"
#include "enums.h"
#include "gpio.h"

// Example call-site for the beamformer library.
// The actual MCU entry point is CubeMX-generated (typically `Core/Src/main.c`).
// Call `App_BeamformerDemo()` after `MX_GPIO_Init()` and SPI1 init.

#define BF_CS_PORT GPIOA
#define BF_CS_PIN  GPIO_PIN_4

void App_BeamformerDemo(void) {
  BF_Init(BF_CS_PORT, BF_CS_PIN); // CS idle high

  bf_register_frame_t reg = {
      .addr10 = AWMF_REG_MODE,
      .data48 = AWMF_MODE_PARITY_EN | AWMF_MODE_SPI_LDB_EN |
                AWMF_MODE_SPI_FBS_SEL,
  };
  (void)BF_Send60(BF_CS_PORT, BF_CS_PIN, &reg);

  bf_broadcast_frame_t bc = {
      .addr10 = AWMF_REG_BW_TX_A,
      .data48 = 0x000012345678ULL,
  };
  (void)BF_Send62_Broadcast(BF_CS_PORT, BF_CS_PIN, &bc);

  bf_fastbeam_frame_t beam = {
      .tdbs_addr_A = 0x01,
      .tdbs_addr_B = 0x02,
      .fbs_addr_A = 0x055,
      .fbs_addr_B = 0x0AA,
      .is_tx_bank = 1,
  };
  (void)BF_Send34_FastBeam(BF_CS_PORT, BF_CS_PIN, &beam);
}
