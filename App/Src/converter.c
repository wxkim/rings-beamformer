// //
// // Created by Laura  on 6/12/25.
// //

// //#include "awmf210.h"

// #define PRINTOVERUART
// #ifdef PRINTOVERUART
// #include "core_cm33.h"

// int _write(int file, char *ptr, int len) {
//   for (int i = 0; i < len; i++) {
//     ITM_SendChar(*ptr++);
//   }
//   return len;
// }

// /* USER CODE BEGIN PFP */
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// /* USER CODE END PFP */

// /* USER CODE BEGIN WHILE */
// while (1) {
//   printf("Hello World\n\r");
//   HAL_Delay(1000);
//   /* USER CODE END WHILE */

//   …
//       /* USER CODE BEGIN 4 */
//       /**
//        * @brief  Retargets the C library printf function to the USART.
//        *   None
//        * @retval None
//        */
//       PUTCHAR_PROTOTYPE {
//     /* Place your implementation of fputc here */
//     /* e.g. write a character to the USART1 and Loop until the end of
//      * transmission */
//     HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

//     return ch;
//   }

// /* USER CODE END 4 */
// #endif

//   // Initialize awmf210
//   void awmf210_init() {}

//   void somefunction() {

//     static uint8_t first_loop_done =
//         0; // flag to only print debugging messages once at first run
//     uint8_t system_ok = 1; // Assume system is OK unless an error is found

//     // Get the angle of the target: right now it simulates
//     //  a target moving from 0 to 100 degrees and back
//     target_info_t target = getTargetInfo();

//     // Calculate phase settings based off of current angle
//     phase_settings_t phases = calculateBeamformingPhases(&target);

//     // Send settings to awmf210
//     awmf210_configTXPath(phases.lo_phase, phases.if_phase, phases.tx_gain);
//     awmf210_configRXPath(phases.lo_phase_rx, phases.if_phase_rx,
//                          phases.rx_gain);

//     // DEBUGGING MESSAGES BEGIN
//     if (!first_loop_done) {
//       printf("\n--- System Startup Health Check ---\n");

//       // Check target angle validity
//       if (target.angle_deg < 0.0f || target.angle_deg > 100.0f) {
//         printf("WARNING: Target angle out of range: %.2f degrees\n",
//                target.angle_deg);
//         system_ok = 0;
//       }

//       // Check phase code validity
//       if (phases.lo_phase > 7) {
//         printf("ERROR: LO Phase code out of bounds: %u\n", phases.lo_phase);
//         system_ok = 0;
//       }

//       if (phases.if_phase > 7) {
//         printf("ERROR: IF Phase code out of bounds: %u\n", phases.if_phase);
//         system_ok = 0;
//       }

//       if (phases.tx_gain > 7) {
//         printf("ERROR: TX Gain code out of bounds: %u\n", phases.tx_gain);
//         system_ok = 0;
//       }

//       if (system_ok) {
//         printf("System Startup Check: OK\n");
//       }

//       else {
//         printf("System Startup Check: FAILED \n");
//       }

//       printf("------------------------------\n");

//       first_loop_done = 1; // Don't print again
//     }
//     // DEBUGGING MESSAGES END

//     HAL_Delay(100);
//   }

//   /* USER CODE BEGIN 4 */
//   void awmf210_write(uint16_t reg_addr, uint64_t data) {
//     uint8_t buffer[8] = {0};

//     reg_addr &= 0x03FF; // only 10 bits

//     // combine command
//     uint64_t full_command =
//         ((uint64_t)reg_addr << 48) | (data & 0xFFFFFFFFFFFF);

//     // pack command
//     buffer[0] = (full_command >> 56) & 0xFF;
//     buffer[1] = (full_command >> 48) & 0xFF;
//     buffer[2] = (full_command >> 40) & 0xFF;
//     buffer[3] = (full_command >> 32) & 0xFF;
//     buffer[4] = (full_command >> 24) & 0xFF;
//     buffer[5] = (full_command >> 16) & 0xFF;
//     buffer[6] = (full_command >> 8) & 0xFF;
//     buffer[7] = (full_command >> 0) & 0xFF;

//     // send command
//     HAL_SPI_Transmit(&hspi1, buffer, 8, HAL_MAX_DELAY);
//   }

//   // Todo refactor redundancies in this file

//   void awmf210_setIFPhaseTX(uint8_t phase_code) {
//     phase_code &= 0x07;                           // 3-bit range
//     uint64_t data = ((uint64_t)phase_code << 38); // Set bits 40–38
//     awmf210_write(0x004, data);                   // Send
//   }

//   void awmf210_setLOPhaseTX(uint8_t phase_code) {
//     // only the lowest 3 bits
//     phase_code &= 0x07;

//     uint64_t data = ((uint64_t)phase_code << 44);

//     // Send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_setTXGain(uint8_t gain_code) {
//     // 3-bit range (0-7)
//     gain_code &= 0x07;

//     uint64_t data = ((uint64_t)gain_code << 32);

//     // Send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_setIFPhaseRX(uint8_t phase_code) {
//     // phase_code is 3 bits
//     phase_code &= 0x07;

//     // Shift into bits 37–35
//     uint64_t data = ((uint64_t)phase_code << 35);

//     // Send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_setLOPhaseRX(uint8_t phase_code) {
//     // phase_code is 3 bits
//     phase_code &= 0x07;

//     // Shift into bits 43–41
//     uint64_t data = ((uint64_t)phase_code << 41);

//     // Send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_setRXGain(uint8_t gain_code) {
//     // gain_code is 3 bits
//     gain_code &= 0x07;

//     // Shift into bits 31–29
//     uint64_t data = ((uint64_t)gain_code << 29);

//     // Send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_configTXPath(uint8_t lo_phase, uint8_t if_phase,
//                             uint8_t gain_code) {
//     // make inputs to 3 bits
//     lo_phase &= 0x07;
//     if_phase &= 0x07;
//     gain_code &= 0x07;

//     // place each field into bit position
//     uint64_t lo_part = ((uint64_t)lo_phase << 44);

//     // 40–38 for IF Phase
//     uint64_t if_part = ((uint64_t)if_phase << 38);

//     // 34–32 for TX Gain
//     uint64_t gain_part = ((uint64_t)gain_code << 32);

//     // put everything together
//     uint64_t data = lo_part | if_part | gain_part;

//     // send to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_configRXPath(uint8_t lo_phase, uint8_t if_phase,
//                             uint8_t gain_code) {
//     // Clamp to 3 bits
//     lo_phase &= 0x07;
//     if_phase &= 0x07;
//     gain_code &= 0x07;

//     // Shift into correct bit positions
//     //  Bits 43–41 for LO Phase RX
//     uint64_t lo_part = ((uint64_t)lo_phase << 41);

//     // Bits 37–35 for IF Phase RX
//     uint64_t if_part = ((uint64_t)if_phase << 35);

//     // Bits 31–29 for RX Gain
//     uint64_t gain_part = ((uint64_t)gain_code << 29);

//     // Combine into one 48-bit word
//     uint64_t data = lo_part | if_part | gain_part;

//     // Write to register 0x004
//     awmf210_write(0x004, data);
//   }

//   void awmf210_init(void) {
//     // Initialize the TX path
//     awmf210_configTXPath(0b000, 0b000, 0b011);
//     // Initialize the RX path
//     awmf210_configRXPath(0b000, 0b000, 0b011);
//   }

//   TargetInfo getTargetInfo(void) {
//     static float angle = 0.0f; // Static variable keeps its value between calls
//     static int8_t direction = 1; // +1 for increasing, -1 for decreasing

//     TargetInfo target;
//     target.angle_deg = angle;

//     // Update angle for next call
//     angle += direction * 1.0f; // Move by 1 degree each time

//     // Reverse direction if we hit limits
//     if (angle >= 100.0f) {
//       direction = -1;
//       angle = 100.0f;
//     } else if (angle <= 0.0f) {
//       direction = 1;
//       angle = 0.0f;
//     }

//     return target;
//   }

//   PhaseSettings calculateBeamformingPhases(TargetInfo target) {
//     PhaseSettings settings;

//     // 1. Calculate LO Phase based on angle
//     // Assume angle maps linearly between 0 degrees -> 0 phase code
//     // and 100 degrees -> 7 phase code
//     float normalized = target.angle_deg / 100.0f;             // 0.0 to 1.0
//     uint8_t phase_code = (uint8_t)(normalized * 7.0f + 0.5f); // rounding

//     // Clamp phase code to 0–7
//     if (phase_code > 7)
//       phase_code = 7;

//     settings.lo_phase = phase_code; // LO Phase for TX
//     settings.if_phase = 0;          // For now, IF phase = 0
//     settings.tx_gain = 3;           // Mid-gain setting (can be dynamic later)

//     settings.lo_phase_rx = phase_code; // Mirror TX LO phase to RX for now
//     settings.if_phase_rx = 0;          // RX IF phase = 0
//     settings.rx_gain = 3;              // RX mid-gain

//     return settings;
//   }

//   /* USER CODE END 4 */

//   /**
//    * @brief  This function is executed in case of error occurrence.
//    * @retval None
//    */
//   void Error_Handler(void) {
//     /* USER CODE BEGIN Error_Handler_Debug */
//     /* User can add his own implementation to report the HAL error return state
//      */
//     __disable_irq();
//     while (1) {
//     }
//     /* USER CODE END Error_Handler_Debug */
//   }

// #ifdef USE_FULL_ASSERT
//   /**
//    * @brief  Reports the name of the source file and the source line number
//    *         where the assert_param error has occurred.
//    * @param  file: pointer to the source file name
//    * @param  line: assert_param error line source number
//    * @retval None
//    */
//   void assert_failed(uint8_t *file, uint32_t line) {
//     /* USER CODE BEGIN 6 */
//     /* User can add his own implementation to report the file name and line
//        number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
//        file, line) */
//     /* USER CODE END 6 */
//   }
// #endif /* USE_FULL_ASSERT */
