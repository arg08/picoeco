cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_eco C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(netmon
	netmon.c
	crc16.c
	econet_ll.c)

# This is a small program, avoid flash performance issues
pico_set_binary_type(netmon copy_to_ram)

# Get the UF2 file built
pico_add_extra_outputs(netmon)

pico_generate_pio_header(netmon ${CMAKE_CURRENT_LIST_DIR}/netmon.pio)

pico_enable_stdio_usb(netmon 1)
# Standard speed UART is too slow
pico_enable_stdio_uart(netmon 0)

target_link_libraries(netmon
        pico_stdlib
        hardware_pio
		hardware_pwm
        )


