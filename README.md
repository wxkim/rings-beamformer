# STM32U575 USB-to-SPI Bridge for AWMF-0165

This project implements a simple USB-to-SPI bridge on the STM32U575 using CDC class USB and SPI communication. It’s designed to control the **Analog Devices AWMF-0165 beamforming IC** over SPI.

---

## Features

- USB CDC interface receives 2-byte commands from PC
- Converts them to SPI writes to the AWMF-0165
- Echoes response back over USB (optional)
- Handles SPI and USB initialization via STM32 HAL

---

## Hardware

- STM32U575 Nucleo or custom board
- AWMF-0165 beamformer IC
- USB connection to host PC
- SPI lines: SCK, MOSI, and software-controlled CS (PA4)

---

## Notes

- SPI polarity/phase configured for AWMF compatibility
- Code assumes CDC_Receive_FS is used as USB callback
- SPI BaudRatePrescaler set to 16 (adjust as needed)

---

## To Use

- Build in STM32CubeIDE
- Connect via USB
- Send 2-byte SPI commands over virtual COM port

---

## Files

- `main.c` – All code in one file for portability

---
