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

file(STRINGS "${CMAKE_SOURCE_DIR}/VERSION" F407_FIRMWARE_VERSION LIMIT_COUNT 1)
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE F407_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT F407_GIT_SHA)
    set(F407_GIT_SHA "unknown")
endif()
execute_process(
    COMMAND git status --porcelain --untracked-files=no
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE F407_GIT_STATUS
    ERROR_QUIET
)
if(F407_GIT_STATUS)
    set(F407_BUILD_DIRTY 1)
else()
    set(F407_BUILD_DIRTY 0)
endif()
configure_file(cmake/build_identity.h.in generated/build_identity.h @ONLY)

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

# Add user-maintained sources to executable
set(F407_APP_BSP_SOURCES
    App/chassis/chassis_control.c
    App/chassis/chassis_maintenance.c
    App/chassis/chassis_math.c
    App/chassis/straight_controller.c
    App/chassis/chassis_task_timing.c
    App/chassis/chassis_tasks.c
    App/control/control_manager.c
    App/control/line_control.c
    App/control/line_calibration.c
    App/control/ps2_control.c
    App/control/relative_yaw_control.c
    App/current/current_guard.c
    App/debug/reset_trace.c
    App/debug/debug_maintenance_policy.c
    App/debug/debug_log_policy.c
    App/debug/debug_straight_telemetry.c
    App/debug/usart1_debug_console.c
    App/display/oled_ui.c
    App/display/oled_selfcheck.c
    App/display/oled_calibration_view.c
    App/imu/imu_calibration_gate.c
    App/monitor/system_monitor.c
    App/monitor/power_on_self_test.c
    App/monitor/reset_reason.c
    App/param/param_store.c
    App/protocol/esp12f_comm.c
    App/protocol/upper_protocol.c
    App/protocol/upper_uart.c
    App/protocol/uart_callbacks.c
    BSP/adc/adc_monitor.c
    BSP/chassis_layout.c
    BSP/encoder/encoder_driver.c
    BSP/encoder/encoder_math.c
    BSP/esp12f/esp12f_flash_bridge.c
    BSP/flash/flash_param.c
    BSP/imu/imu_bmi270.c
    BSP/imu/imu_bmi270_calibration.c
    BSP/imu/imu_bmi270_config.c
    BSP/imu/imu_bmi270_fifo.c
    BSP/imu/imu_bmi270_math.c
    BSP/imu/imu_bmi270_profile.c
    BSP/imu/imu_bmi270_time.c
    BSP/led/led_status.c
    BSP/line/line_uart.c
    BSP/motor/motor_driver.c
    BSP/motor/motor_output_logic.c
    BSP/pid/pid_controller.c
    BSP/ps2/ps2_hw.c
    BSP/oled/ssd1306.c
    BSP/oled/oled_font_data.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${F407_APP_BSP_SOURCES})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    App/chassis
    App/control
    App/current
    App/debug
    App/display
    App/imu
    App/monitor
    App/param
    App/protocol
    BSP
    BSP/adc
    BSP/encoder
    BSP/esp12f
    BSP/flash
    BSP/imu
    BSP/led
    BSP/line
    BSP/motor
    BSP/pid
    BSP/ps2
    BSP/oled
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:DEBUG_CONSOLE_RELEASE_REQUIRES_ARM=1>
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set_property(SOURCE ${F407_APP_BSP_SOURCES} APPEND PROPERTY COMPILE_OPTIONS
        -Wall
        -Wextra
        -Wpedantic
        -Werror
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex/.bin and printing memory usage"
)
