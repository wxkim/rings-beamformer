cmake_minimum_required(VERSION 3.22)

# Project
set(CMAKE_PROJECT_NAME anoki)
project(${CMAKE_PROJECT_NAME} C ASM)

# Compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Create executable
add_executable(${CMAKE_PROJECT_NAME})

# Sources (add all your firmware sources here)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Application
    ${CMAKE_SOURCE_DIR}/App/Src/beamformer.c
    ${CMAKE_SOURCE_DIR}/App/Src/usb-spi-control.c

    # Core
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Core/Src/spi.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32u5xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32u5xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32u5xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c

    # HAL Drivers
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c

    # Startup
    ${CMAKE_SOURCE_DIR}/startup_stm32u575xx.s
)

# Includes
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32U5xx/Include
)

# Preprocessor symbols
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    STM32U575xx
    USE_HAL_DRIVER
)

# CPU/FPU flags (very important for Cortex-M33)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/STM32U575xx_FLASH.ld   # linker script
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)

# Remove wrong libob.a if it sneaks in
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
