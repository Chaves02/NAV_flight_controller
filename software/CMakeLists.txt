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

# Add source files for libraries
add_library(drone_leds
        ${CMAKE_CURRENT_SOURCE_DIR}/include/led/led.cpp
)

target_link_libraries(drone_leds
        pico_stdlib
)

add_library(ahr
        ${CMAKE_CURRENT_SOURCE_DIR}/include/ahr/ahr.cpp
)

target_link_libraries(ahr
        pico_stdlib
        rp_agrolib_bmi088
)

add_library(out
        ${CMAKE_CURRENT_SOURCE_DIR}/include/out/out.cpp
)

target_link_libraries(out
        pico_stdlib
)

add_library(rclsim
        ${CMAKE_CURRENT_SOURCE_DIR}/include/rclsim/rclsim.cpp
)

target_link_libraries(rclsim
        pico_stdlib
)

add_library(pid
        ${CMAKE_CURRENT_SOURCE_DIR}/include/pid/pid.cpp
)

target_link_libraries(pid
        pico_stdlib
)

add_library(madflight
        ${CMAKE_CURRENT_SOURCE_DIR}/include/madflight/madflight.cpp
)

target_link_libraries(madflight
        pico_stdlib
        drone_leds
        ahr
        out
        pid
        rclsim
        rp_agrolib_i2c
        rp_agrolib_bmi088
        rp_agrolib_vl53l8cx
)

############################################################
# MAIN
############################################################

add_executable(nav_flight_controller
        src/main.cpp)

pico_enable_stdio_usb(nav_flight_controller 1)
pico_enable_stdio_uart(nav_flight_controller 0)

pico_add_extra_outputs(nav_flight_controller)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(nav_flight_controller
        pico_stdlib
        rp_agrolib_motors
        rp_agrolib_vl53l8cx
        rp_agrolib_bmi088
        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
        rp_agrolib_i2c
)

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
# Altitude_mode
################################################################

add_executable(altitude_mode
        src/altitude_mode.cpp)

pico_enable_stdio_usb(altitude_mode 1)
pico_enable_stdio_uart(altitude_mode 0)

pico_add_extra_outputs(altitude_mode)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(altitude_mode
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
)

################################################################
# Altitude_mode
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
# docking
#################################################################

add_executable(docking
        src/docking.cpp)

pico_enable_stdio_usb(docking 1)
pico_enable_stdio_uart(docking 0)

pico_add_extra_outputs(docking)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(docking

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
# docking_spi
#################################################################

add_executable(docking_spi
        src/docking_spi.cpp)

pico_enable_stdio_usb(docking_spi 1)
pico_enable_stdio_uart(docking_spi 0)

pico_add_extra_outputs(docking_spi)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(docking_spi

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

############################################################
#Quadcopter Flight Controller
############################################################

add_executable(quadcopter_flight_controller
        src/Quadcopter.cpp)

pico_enable_stdio_usb(quadcopter_flight_controller 1)
pico_enable_stdio_uart(quadcopter_flight_controller 0)

pico_add_extra_outputs(quadcopter_flight_controller)

# Link libraries - add the specific driver and interface libraries you need
target_link_libraries(quadcopter_flight_controller
        
        pico_stdlib

        rp_agrolib_vl53l8cx
        rp_agrolib_bmi088
        rp_agrolib_i2c


        # Local libraries
        drone_leds
        ahr
        out
        rclsim
        pid
        madflight

        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_adc
)
