# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040_canbus CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rp2040_rc_control C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rp2040_rc_control rp2040_rc_control.cpp capture_edge.c)

pico_set_program_name(rp2040_rc_control "rp2040_rc_control")
pico_set_program_version(rp2040_rc_control "0.1")

# Generate PIO header
pico_generate_pio_header(rp2040_rc_control ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(rp2040_rc_control ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)  # NEOPIXEL 
pico_generate_pio_header(rp2040_rc_control ${CMAKE_CURRENT_LIST_DIR}/ppm.pio)  # PPM for RemoteControl
#pico_generate_pio_header(rp2040_rc_control ${CMAKE_CURRENT_LIST_DIR}/PwmIn.pio)  # PPM for RemoteControl
pico_generate_pio_header(rp2040_rc_control ${CMAKE_CURRENT_LIST_DIR}/capture_edge.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rp2040_rc_control 0)
pico_enable_stdio_usb(rp2040_rc_control 1)

# Add the standard library to the build
target_link_libraries(rp2040_rc_control
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(rp2040_rc_control PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(rp2040_rc_control 
        hardware_dma
        hardware_irq
        hardware_pio
        hardware_pwm
        hardware_gpio
        hardware_clocks
        hardware_timer
        hardware_watchdog
        )

target_compile_definitions(rp2040_rc_control PRIVATE PARAM_ASSERTIONS_ENABLED_PIO=1)

pico_add_extra_outputs(rp2040_rc_control)

