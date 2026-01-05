cmake_minimum_required(VERSION 3.22)

# Project setup
project(anoki C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()


# Automatically gather all source files
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/App/Src/*.c
    ${CMAKE_SOURCE_DIR}/Core/Src/*.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/*.c
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32U5xx_Nucleo/*.c
)

set(STARTUP_FILE ${CMAKE_SOURCE_DIR}/startup_stm32u575xx.s)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32U575xx_FLASH.ld CACHE FILEPATH "Linker script to use when linking the target image")

# Create the executable target
add_executable(${PROJECT_NAME} ${APP_SOURCES} ${STARTUP_FILE})


# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32U5xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32U5xx_Nucleo
)


target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32U575xx
    USE_HAL_DRIVER
)


target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -Wall
    -ffunction-sections
    -fdata-sections
    $<$<CONFIG:Debug>:-O0 -g3>
    $<$<CONFIG:Release>:-Os -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g3>
    $<$<CONFIG:MinSizeRel>:-Os -DNDEBUG>
)

target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
