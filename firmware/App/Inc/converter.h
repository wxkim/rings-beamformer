#ifndef AWMF210_H
#define AWMF210_H

// #include "core_cm33.h"
#include "main.h"
#include "spi.h"
#include "stdio.h"

#define _PRINT_OVER_UART // Comment this out when debugging is no longer
                         // necessary
#ifdef _PRINT_OVER_UART

// THIS IS A SYSCALL OVERRIDE

int _write(int file, char *ptr, int len) {
  for (int i = 0; i < len; i++)
    ITM_SendChar(*ptr++);
  return len;
}

#endif

typedef struct {
  float angle_deg;
} target_info_t; // TargetInfo

typedef struct {
  uint8_t lo_phase_tx; // local oscillator
  uint8_t if_phase_tx; // intermediate frequency
  uint8_t gain_tx;     // tx gain
  uint8_t lo_phase_rx;
  uint8_t if_phase_rx;
  uint8_t gain_rx;
} phase_settings_t; // PhaseSettings

void awmf210_write(uint16_t reg_addr, uint64_t data);

void awmf210_setIFPhaseTX(uint8_t phase_code);

void awmf210_setLOPhaseTX(uint8_t phase_code);

void awmf210_setTXGain(uint8_t gain_code);

void awmf210_setIFPhaseRX(uint8_t phase_code);

void awmf210_setLOPhaseRX(uint8_t phase_code);

void awmf210_setRXGain(uint8_t gain_code);

void awmf210_configTXPath(uint8_t lo_phase, uint8_t if_phase,
                          uint8_t gain_code);

void awmf210_configRXPath(uint8_t lo_phase, uint8_t if_phase,
                          uint8_t gain_code);

void awmf210_init(void);

target_info_t getTargetInfo(void);

phase_settings_t calculateBeamformingPhases(target_info_t *target);

#endif
