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

# Five-layer user-maintained sources
set(F407_APP_SOURCES
    App/app_init.c
    App/app_isr.c
    App/tasks/task_debug.c
    App/tasks/task_esp12f.c
    App/tasks/task_imu.c
    App/tasks/task_led.c
    App/tasks/task_line.c
    App/tasks/task_motor.c
    App/tasks/task_oled.c
    App/tasks/task_ps2.c
    App/tasks/task_safety.c
    App/tasks/task_upper_comm.c
    App/debug/debug_maintenance_policy.c
    App/debug/debug_log_policy.c
    App/debug/debug_straight_telemetry.c
    App/debug/usart1_debug_console.c
    App/display/oled_ui.c
    App/display/oled_selfcheck.c
    App/display/oled_calibration_view.c
    App/protocol/uart_callbacks.c
    App/protocol/esp12f_flash_bridge.c
)

set(F407_PLATFORM_SOURCES
    Platform/platform_critical.c
    Platform/platform_reset.c
    Platform/platform_reset_trace.c
    Platform/platform_task_event.c
    Platform/platform_time.c
    Platform/platform_watchdog.c
)

set(F407_SERVICE_SOURCES
    Service/chassis/chassis_service.c
    Service/chassis/chassis_maintenance_service.c
    Service/diagnostics/task_health_service.c
    Service/control/control_service.c
    Service/control/line_control_service.c
    Service/control/ps2_control_service.c
    Service/safety/current_guard.c
    Service/safety/safety_service.c
    Service/safety/post_service.c
    Service/diagnostics/reset_reason_service.c
    Service/param/param_service.c
    Service/param/param_persistence.c
    Service/param/flash_param.c
    Service/imu/imu_service.c
    Service/imu/imu_calibration_service.c
    Service/sensing/current_sensor_service.c
    Service/sensing/encoder_service.c
    Service/communication/esp12f_service.c
    Service/communication/upper_uart_service.c
)

set(F407_DOMAIN_SOURCES
    Domain/chassis/chassis_kinematics.c
    Domain/chassis/chassis_layout.c
    Domain/control/pid_controller.c
    Domain/control/relative_yaw_controller.c
    Domain/control/straight_controller.c
    Domain/encoder/encoder_math.c
    Domain/imu/imu_calibration.c
    Domain/imu/imu_calibration_gate.c
    Domain/imu/imu_timestamp.c
    Domain/line/line_calibration.c
    Domain/motor/motor_output_logic.c
    Domain/protocol/upper_protocol.c
)

set(F407_BSP_SOURCES
    BSP/adc/adc_monitor.c
    BSP/encoder/encoder_driver.c
    BSP/flash/flash_storage.c
    BSP/imu/imu_bmi270.c
    BSP/imu/imu_bmi270_config.c
    BSP/imu/imu_bmi270_fifo.c
    BSP/imu/imu_bmi270_math.c
    BSP/imu/imu_bmi270_profile.c
    BSP/led/led_status.c
    BSP/line/line_uart.c
    BSP/motor/motor_driver.c
    BSP/ps2/ps2_hw.c
    BSP/transport/debug_uart_transport.c
    BSP/transport/esp12f_transport.c
    BSP/transport/upper_uart_transport.c
    BSP/oled/ssd1306.c
    BSP/oled/oled_font_data.c
)

set(F407_LAYER_INCLUDE_DIRS
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    App
    App/tasks
    App/chassis
    App/control
    App/current
    App/debug
    App/display
    App/imu
    App/monitor
    App/param
    App/protocol
    Platform
    Service/chassis
    Service/communication
    Service/control
    Service/diagnostics
    Service/imu
    Service/param
    Service/safety
    Service/sensing
    Domain/chassis
    Domain/config
    Domain/control
    Domain/encoder
    Domain/imu
    Domain/line
    Domain/model
    Domain/motor
    Domain/protocol
    Domain/safety
    BSP
    BSP/adc
    BSP/encoder
    BSP/esp12f
    BSP/flash
    BSP/imu
    BSP/led
    BSP/line
    BSP/motor
    BSP/ps2
    BSP/transport
    BSP/oled
)

add_library(f407_domain STATIC ${F407_DOMAIN_SOURCES})
add_library(f407_platform STATIC ${F407_PLATFORM_SOURCES})
add_library(f407_bsp STATIC ${F407_BSP_SOURCES})
add_library(f407_service STATIC ${F407_SERVICE_SOURCES})
add_library(f407_app STATIC ${F407_APP_SOURCES})

foreach(layer_target f407_domain f407_platform f407_bsp f407_service f407_app)
    target_include_directories(${layer_target} PRIVATE ${F407_LAYER_INCLUDE_DIRS})
endforeach()
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${F407_LAYER_INCLUDE_DIRS})

target_link_libraries(f407_domain PUBLIC m)
target_link_libraries(f407_platform PRIVATE stm32cubemx)
target_link_libraries(f407_bsp PRIVATE f407_domain f407_platform stm32cubemx)
target_link_libraries(f407_service PRIVATE f407_domain f407_bsp f407_platform)
target_link_libraries(f407_app PRIVATE f407_service f407_bsp f407_platform stm32cubemx)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:DEBUG_CONSOLE_RELEASE_REQUIRES_ARM=1>
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    foreach(layer_target f407_domain f407_platform f407_bsp f407_service f407_app)
        target_compile_options(${layer_target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endforeach()
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    f407_app
    f407_service
    f407_domain
    f407_bsp
    f407_platform
    f407_domain
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex/.bin and printing memory usage"
)
