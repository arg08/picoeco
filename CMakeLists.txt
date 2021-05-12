cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_eco C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(ecotest ecotest.c econet_ll.c)

# Get the UF2 file built
pico_add_extra_outputs(ecotest)

pico_generate_pio_header(ecotest ${CMAKE_CURRENT_LIST_DIR}/econet.pio)

pico_enable_stdio_usb(ecotest 1)

target_link_libraries(ecotest
        pico_stdlib
        hardware_pio
		hardware_pwm
        )


