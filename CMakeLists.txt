cmake_minimum_required(VERSION 3.22)

# Project setup
set(CMAKE_PROJECT_NAME anoki)
project(${CMAKE_PROJECT_NAME} C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()


# Automatically gather all source files
file(GLOB_RECURSE APP_SOURCES
    ${CMAKE_SOURCE_DIR}/App/Src/*.c
    ${CMAKE_SOURCE_DIR}/Core/Src/*.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/*.c
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32U5xx_Nucleo/*.c
)

set(STARTUP_FILE ${CMAKE_SOURCE_DIR}/startup_stm32u575xx.s)

# Create the executable target
add_executable(${CMAKE_PROJECT_NAME} ${APP_SOURCES} ${STARTUP_FILE})


# Include directories
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32U5xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32U5xx_Nucleo
)


target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    STM32U575xx
    USE_HAL_DRIVER
)


target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -Wall
    -ffunction-sections
    -fdata-sections
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    #-T${CMAKE_SOURCE_DIR}/STM32U575xx_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)


list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
