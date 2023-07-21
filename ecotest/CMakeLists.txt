cmake_minimum_required(VERSION 3.12)

# Board can be specified on the command line, otherwise assume a default
if(NOT DEFINED PICO_BOARD)
	set(PICO_BOARD eco_skt_box)
elseif(PICO_BOARD STREQUAL "pico")
	message(NOTICE "PICO_BOARD should be defined in environment or with -D")
else()
	message(NOTICE "PICO_BOARD is defined: " ${PICO_BOARD})
endif()


set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_eco C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


if(PICO_BOARD STREQUAL "eco_skt_box")
set(MAIN_EXECUTABLE eco_skt_box)
add_executable(${MAIN_EXECUTABLE}
	ecotest.c
	board_specific.c
	crc16.c
	timers.c
	fs.c
	econet_hl.c
	econet_ll.c)

elseif(PICO_BOARD STREQUAL "bus1mhz_proto")
set(MAIN_EXECUTABLE bus1mhz_proto)
add_executable(${MAIN_EXECUTABLE}
	ecotest.c
	board_specific.c
	bus1mhz.c
	crc16.c
	timers.c
	fs.c
	econet_hl.c
	econet_ll.c)
else()
	message(ERROR "No executable for this board in CMakeLists.txt")
endif()

# All boards need this
pico_generate_pio_header(${MAIN_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/econet.pio)

# Only 1MHz boards need this
if(PICO_BOARD STREQUAL "bus1mhz_proto")
pico_generate_pio_header(${MAIN_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/bus1mhzproto.pio)
endif()

# Get the UF2 file built
pico_add_extra_outputs(${MAIN_EXECUTABLE})

set(PICO_UART_DEFAULT_BAUD 2000000)
pico_enable_stdio_usb(${MAIN_EXECUTABLE} 1)
pico_enable_stdio_uart(${MAIN_EXECUTABLE} 1)


# Not all executables need all these libararies, but there's no difference
# to the executable by including them unconditionally
target_link_libraries(${MAIN_EXECUTABLE}
        pico_stdlib
        hardware_pio
		hardware_pwm
		hardware_dma
		hardware_spi
        )


