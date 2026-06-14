cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME F407_V2.0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/chassis/chassis_control.c
    App/chassis/chassis_layout.c
    App/chassis/chassis_math.c
    App/chassis/chassis_task_timing.c
    App/chassis/chassis_tasks.c
    App/control/control_manager.c
    App/debug/usart1_debug_console.c
    App/monitor/system_monitor.c
    App/protocol/upper_protocol.c
    App/protocol/upper_uart.c
    App/protocol/uart_callbacks.c
    BSP/adc/adc_monitor.c
    BSP/encoder/encoder_driver.c
    BSP/encoder/encoder_math.c
    BSP/esp12f/esp12f_comm.c
    BSP/esp12f/esp12f_flash_bridge.c
    BSP/imu/imu_bmi270.c
    BSP/imu/imu_bmi270_config.c
    BSP/led/led_status.c
    BSP/line/line_uart.c
    BSP/motor/motor_driver.c
    BSP/pid/pid_controller.c
    BSP/ps2/ps2_control.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/chassis
    App/control
    App/debug
    App/monitor
    App/protocol
    BSP/adc
    BSP/encoder
    BSP/esp12f
    BSP/imu
    BSP/led
    BSP/line
    BSP/motor
    BSP/pid
    BSP/ps2
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
