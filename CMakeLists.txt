cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

set(PICO_BOARD pico_w)

pico_sdk_init()

project(core_temp C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


add_executable(TEMP_MONITOR main.c)

target_link_libraries(TEMP_MONITOR pico_cyw43_arch_none pico_stdlib)
target_link_libraries(TEMP_MONITOR pico_stdlib hardware_adc)
target_link_libraries(TEMP_MONITOR pico_stdlib hardware_i2c)

pico_enable_stdio_usb(TEMP_MONITOR 1)
pico_enable_stdio_uart(TEMP_MONITOR 0)

pico_add_extra_outputs(TEMP_MONITOR)
