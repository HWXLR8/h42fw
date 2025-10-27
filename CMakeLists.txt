cmake_minimum_required(VERSION 3.13)

# pico sdk
set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/pico-sdk CACHE PATH "")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(h42fw C CXX ASM)
pico_sdk_init()

add_executable(h42fw
    src/main.c
    src/usb_descriptors.c
)
set_target_properties(h42fw PROPERTIES SUFFIX ".elf")

target_include_directories(h42fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(h42fw
    pico_stdlib
    tinyusb_device
    tinyusb_board
    hardware_gpio
)

pico_enable_stdio_usb(h42fw 1)
pico_enable_stdio_uart(h42fw 0)
pico_add_extra_outputs(h42fw)
