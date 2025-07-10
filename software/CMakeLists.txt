cmake_minimum_required(VERSION 3.16)

include(pico_sdk_import.cmake)

## Set the path to your Pico SDK
#set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk")
#
## Include the Pico SDK import script
#include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(NAV_flight_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Find linker paths
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/drivers/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/interfaces/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/third-party/)

# Find include paths
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/drivers/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/interfaces/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../extern/rp2040_agrolib/software/build/interfaces/third-party/)
include_directories(include)

# Add Pico SDK include paths
include_directories(${PICO_SDK_PATH}/src/common/pico_stdlib/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/pico_platform/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/hardware_i2c/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/hardware_adc/include)
include_directories(${PICO_SDK_PATH}/src/rp2_common/pico_time/include)

################################################################
# Angle_mode
################################################################

add_executable(angle_mode
        src/angle_mode.cpp)

pico_enable_stdio_usb(angle_mode 1)
pico_enable_stdio_uart(angle_mode 0)

pico_add_extra_outputs(angle_mode)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(angle_mode
        pico_stdlib
        pico_time
        pico_multicore
        rp_agrolib_motors
        rp_agrolib_bmi088
        rp_agrolib_uart
        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
        rp_agrolib_i2c
)

################################################################
# Velocity_mode
################################################################

add_executable(velocity_mode
        src/velocity_mode.cpp)

pico_enable_stdio_usb(velocity_mode 1)
pico_enable_stdio_uart(velocity_mode 0)

pico_add_extra_outputs(velocity_mode)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(velocity_mode
        pico_stdlib
        pico_time
        pico_multicore
        rp_agrolib_motors
        rp_agrolib_vl53l8cx
        rp_agrolib_bmi088
        rp_agrolib_uart
        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
        rp_agrolib_i2c
        rp_agrolib_bc832
)

################################################################
# docking_control
#################################################################

add_executable(docking_control
        src/docking_control.cpp)

pico_enable_stdio_usb(docking_control 1)
pico_enable_stdio_uart(docking_control 0)

pico_add_extra_outputs(docking_control)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(docking_control
        pico_stdlib
        pico_stdio_usb
        pico_time
        pico_multicore

        rp_agrolib_bc832
)

################################################################
# docking_monochrome
#################################################################

add_executable(docking_monochrome
        src/docking_monochrome.cpp)

pico_enable_stdio_usb(docking_monochrome 1)
pico_enable_stdio_uart(docking_monochrome 0)

pico_add_extra_outputs(docking_monochrome)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(docking_monochrome

        pico_stdlib
        pico_stdio_usb
        pico_time
        pico_multicore

        rp_agrolib_uart
        rp_agrolib_i2c

        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
        hardware_spi
        hardware_pio
        hardware_dma
)

################################################################
# docking_spicolor
#################################################################

add_executable(docking_spicolor
        src/docking_spicolor.cpp)

pico_enable_stdio_usb(docking_spicolor 1)
pico_enable_stdio_uart(docking_spicolor 0)

pico_add_extra_outputs(docking_spicolor)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(docking_spicolor

        pico_stdlib
        pico_stdio_usb
        pico_time
        pico_multicore

        rp_agrolib_uart
        rp_agrolib_i2c
        rp_agrolib_bc832

        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
        hardware_spi
        hardware_pio
        hardware_dma
)
