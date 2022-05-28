cmake_minimum_required(VERSION 3.12)

# Board can be specified on the command line, otherwise assume a default
if(NOT DEFINED PICO_BOARD)
	set(PICO_BOARD eco_skt_box)
elseif(PICO_BOARD EQUAL "pico")
	message(NOTICE "PICO_BOARD should be defined in environment or with -D")
else()
	message(NOTICE "PICO_BOARD is defined" ${PICO_BOARD})
endif()


set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_eco C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Werror")

# Initialize the SDK
pico_sdk_init()


add_executable(ecotest
	ecotest.c
	board_specific.c
	crc16.c
	timers.c
	fs.c
	econet_hl.c
	econet_ll.c)

# Get the UF2 file built
pico_add_extra_outputs(ecotest)

pico_generate_pio_header(ecotest ${CMAKE_CURRENT_LIST_DIR}/econet.pio)

set(PICO_UART_DEFAULT_BAUD 2000000)
pico_enable_stdio_usb(ecotest 1)
pico_enable_stdio_uart(ecotest 1)

target_link_libraries(ecotest
        pico_stdlib
        hardware_pio
		hardware_pwm
		hardware_spi
        )


