cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
#link_directories(ext_libraries)

add_subdirectory(/home/eesh/micro_ros_ws/src/pico-arduino-compat/pico-libs/pico-adafruit-gfx-library build)
add_subdirectory(/home/eesh/micro_ros_ws/src/pico-arduino-compat/libs-extra/pico-st7735 build_pico-st7735)
#add_subdirectory(/home/eesh/micro_ros_ws/src/pico-arduino-compat/libs/adafruit-st7735-and-st7789-library build_pac-adafruit-st7735-and-st7789-library)

add_executable(pico_micro_ros_example
    #pico_micro_ros_example.c
    pico_uart_transport.c
    #custom_pico_micro_ros.c
    final_custom_pico_micro_ros.c

)
target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
    #pac-adafruit-st7735-and-st7789-library
    pico-adafruit-gfx-library
    pico-st7735
)

target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include
    #ext_libraries/include/Adafruit-ST7735-Library
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_definitions(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=5000)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
