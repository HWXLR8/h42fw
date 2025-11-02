cmake_minimum_required(VERSION 3.13)

# pico sdk
set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/pico-sdk CACHE PATH "")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(h42fw C CXX ASM)
pico_sdk_init()

add_executable(h42fw
    src/main.c
    src/oled.c
    src/usb_descriptors.c
)
set_target_properties(h42fw PROPERTIES SUFFIX ".elf")
pico_generate_pio_header(h42fw ${CMAKE_CURRENT_LIST_DIR}/lib/ws2812.pio)
target_include_directories(h42fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(h42fw
    pico_stdlib
    pico_multicore
    pico_util
    tinyusb_device
    tinyusb_board
    hardware_gpio
    hardware_pio
    hardware_i2c
)

pico_enable_stdio_usb(h42fw 1)
pico_enable_stdio_uart(h42fw 0)
pico_add_extra_outputs(h42fw)
