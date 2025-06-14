# STM32U575 USB-to-SPI Bridge for AWMF-0165

The STM32U575 acts as a bridge between the computer and the AWMF-0165 beamforming chip. It receives the USB data from the computer and sends instrutions to the AWMF-01665 chip using SPI commands.

---

## Features

- PC can communicate with SPI device using USB
- Uses AWMF-0165 chip
- Settings through STM32CubeMax

---

## Hardware

- STM32U575 Nucleo or custom board
- AWMF-0165 beamformer IC
- USB connection to host PC

---

## To Use

- Open Project through STM32CubeIDE
- Build and Flash STM32 board
- Connect PC to USB
- Open terminal to send message

---

## Files

- `Core/` – Main C code 
- 'Drivers/ - STM32 and USB libraries
- 'usb_spi_bridge.c' - USB to SPI bridge code
- 'rings-beamformer.ioc' - STM32CubeMX project file
- 'CMakeLists.txt' - CMake build file
- `STM32U575xx_FLASH.ld` - Flash Memory Script
- `STM32U575xx_RAM.ld` - RAM Memory SCript

---
