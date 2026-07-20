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
    COMMAND git rev-parse HEAD
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
if(F407_GIT_STATUS OR F407_GIT_SHA STREQUAL "unknown")
    set(F407_BUILD_DIRTY 1)
    set(F407_BUILD_IDENTITY_VALID 0)
    set(F407_GIT_COMMIT_BYTES "0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00")
else()
    set(F407_BUILD_DIRTY 0)
    set(F407_BUILD_IDENTITY_VALID 1)
    set(F407_GIT_COMMIT_BYTES "")
    foreach(F407_GIT_BYTE_INDEX RANGE 0 19)
        math(EXPR F407_GIT_HEX_OFFSET "${F407_GIT_BYTE_INDEX} * 2")
        string(SUBSTRING "${F407_GIT_SHA}" ${F407_GIT_HEX_OFFSET} 2 F407_GIT_HEX_BYTE)
        if(F407_GIT_COMMIT_BYTES)
            string(APPEND F407_GIT_COMMIT_BYTES ", ")
        endif()
        string(APPEND F407_GIT_COMMIT_BYTES "0x${F407_GIT_HEX_BYTE}")
    endforeach()
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
    App/composition/firmware_identity_provider.c
    App/config/robot_config.c
    App/composition/system_publish_snapshot_collector.c
    App/composition/operator_action_router.c
    App/composition/line_calibration_orchestrator.c
    App/composition/imu_calibration_orchestrator.c
    App/tasks/task_debug.c
    App/tasks/task_esp12f.c
    App/tasks/task_imu.c
    App/tasks/task_led.c
    App/tasks/task_motor.c
    App/tasks/task_oled.c
    App/tasks/task_ps2.c
    App/tasks/task_safety.c
    App/tasks/task_line.c
    App/tasks/task_upper_comm.c
)

set(F407_APP_ADAPTER_SOURCES
    App/app_isr.c
    App/adapters/app_hardware_init.c
    App/adapters/status_led_adapter.c
    App/debug/debug_maintenance_policy.c
    App/debug/debug_log_policy.c
    App/debug/debug_straight_telemetry.c
    App/debug/debug_straight_telemetry.h
    App/debug/debug_console_parser.c
    App/debug/debug_console_parser.h
    App/debug/debug_console_registry.c
    App/debug/debug_console_writer.c
    App/debug/commands/debug_cmd_param.c
    App/debug/commands/debug_cmd_imu.c
    App/debug/commands/debug_cmd_esp12f.c
    App/debug/commands/debug_cmd_control.c
    App/debug/commands/debug_cmd_line.c
    App/debug/commands/debug_cmd_current.c
    App/debug/commands/debug_cmd_system.c
    App/debug/commands/debug_system_status.c
    App/debug/commands/debug_rtos_report.c
    App/debug/telemetry/debug_telemetry.c
    App/debug/telemetry/debug_telemetry_model.h
    App/debug/telemetry/debug_telemetry_csv.c
    App/debug/telemetry/debug_telemetry_json.c
    App/debug/usart1_debug_console.c
    App/display/oled_ui.c
    App/display/oled_ui_model.c
    App/display/oled_text_renderer.c
    App/display/pages/oled_page_welcome.c
    App/display/pages/oled_page_selfcheck.c
    App/display/pages/oled_page_runtime.c
    App/display/pages/oled_page_calibration.c
    App/display/pages/oled_page_fault.c
    App/display/oled_calibration_view.c
    App/adapters/uart_callbacks.c
    App/adapters/esp12f_flash_bridge.c
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
    Service/motion_control/motion_control_service.c
    Service/motion_control/internal/motion_maintenance_service.c
    Service/motion_control/internal/motion_parameter_sync.c
    Service/motion_control/internal/wheel_target_planner.c
    Service/motion_control/internal/wheel_speed_control_loop.c
    Service/motion_control/internal/wheel_feedback_monitor.c
    Service/motion_control/internal/motor_output_coordinator.c
    Service/motion_control/internal/motion_test_mode.c
    Service/motion_control/internal/motion_status_builder.c
    Service/motion_control/internal/motor_current_limiter.c
    Service/command_management/command_management_service.c
    Service/line_following/line_following_service.c
    Service/line_following/internal/line_parameter_sync.c
    Service/teleoperation/teleoperation_service.c
    Service/safety_management/internal/battery_voltage_monitor.c
    Service/safety_management/internal/fault_stop_policy.c
    Service/safety_management/safety_management_service.c
    Service/safety_management/internal/power_on_self_test.c
    Service/parameter_management/parameter_management_service.c
    Service/parameter_management/internal/parameter_management_persistence.c
    Service/parameter_management/internal/parameter_identity_crc.c
    Service/parameter_management/internal/parameter_persistence_backend.c
    Service/parameter_management/internal/flash_parameter_image.c
    Service/parameter_management/internal/flash_parameter_codec.c
    Service/parameter_management/internal/flash_parameter_migration.c
    Service/parameter_management/internal/flash_parameter_slot_store.c
    Service/power_management/power_management_service.c
    Service/state_estimation/state_estimation_service.c
    Service/parameter_management/parameter_imu_calibration.c
    Service/state_estimation/internal/imu_estimation_pipeline.c
    Service/state_estimation/internal/imu_quality_monitor.c
    Service/state_estimation/internal/imu_calibration_coordinator.c
    Service/state_estimation/internal/wheel_estimation_pipeline.c
    Service/system_monitoring/system_monitoring_service.c
    Service/communication/wireless_communication_service.c
    Service/communication/host_communication_service.c
    Service/communication/internal/remote_command_dispatcher.c
    Service/communication/internal/communication_session_tracker.c
    Service/communication/internal/telemetry_encoder.c
    Service/system_monitoring/internal/system_publish_snapshot.c
    Service/communication/internal/frame_stream_parser.c
    Service/communication/internal/robot_link_protocol.c
)

set(F407_ALGORITHM_SOURCES
    Algorithm/control/wheel_speed_pid_controller.c
    Algorithm/control/relative_heading_controller.c
    Algorithm/control/straight_line_controller.c
    Algorithm/calibration/imu_calibration.c
    Algorithm/calibration/imu_calibration_guard.c
    Algorithm/calibration/line_sensor_calibration.c
    Algorithm/estimation/attitude_estimator.c
    Algorithm/estimation/imu_timestamp_tracker.c
    Algorithm/estimation/wheel_speed_estimator.c
    Algorithm/filtering/imu_signal_filter.c
    Algorithm/kinematics/differential_drive_kinematics.c
)

set(F407_BSP_SOURCES
    BSP/adc/power_adc_driver.c
    BSP/diagnostics/i2c_bus_diagnostic.c
    BSP/encoder/wheel_encoder_driver.c
    BSP/esp12f/esp12f_boot_control.c
    BSP/flash/flash_storage.c
    BSP/imu/bmi270_driver.c
    BSP/imu/bmi270_bus.c
    BSP/imu/bmi270_device.c
    BSP/imu/bmi270_config.c
    BSP/imu/bmi270_fifo_reader.c
    BSP/imu/bmi270_profile.c
    BSP/led/status_led_driver.c
    BSP/line/line_sensor_driver.c
    BSP/motor/motor_driver.c
    BSP/motor/motor_break.c
    BSP/motor/motor_hardware_map.c
    BSP/motor/internal/motor_hardware_layout.c
    BSP/motor/internal/motor_ph_en_mapper.c
    BSP/motor/internal/motor_reversal_state_machine.c
    BSP/ps2/ps2_controller_driver.c
    BSP/transport/debug_uart_transport.c
    BSP/transport/esp12f_transport.c
    BSP/transport/uart_bridge_transport.c
    BSP/transport/host_uart_transport.c
    BSP/oled/ssd1306.c
    BSP/oled/oled_font_data.c
)

add_library(f407_algorithm STATIC ${F407_ALGORITHM_SOURCES})
add_library(f407_platform STATIC ${F407_PLATFORM_SOURCES})
add_library(f407_bsp STATIC ${F407_BSP_SOURCES})
add_library(f407_service STATIC ${F407_SERVICE_SOURCES})
add_library(f407_app STATIC ${F407_APP_SOURCES})
add_library(f407_app_adapters STATIC ${F407_APP_ADAPTER_SOURCES})

target_include_directories(f407_algorithm PUBLIC
    Algorithm/control
    Algorithm/estimation
    Algorithm/calibration
    Algorithm/kinematics
    Algorithm/filtering
)

target_include_directories(f407_platform
    PUBLIC Platform
    PRIVATE Core/Inc
)

target_include_directories(f407_bsp
    PUBLIC
        BSP
        BSP/adc
        BSP/diagnostics
        BSP/encoder
        BSP/esp12f
        BSP/flash
        BSP/imu
        BSP/led
        BSP/line
        BSP/motor
        BSP/oled
        BSP/ps2
        BSP/transport
    PRIVATE
        BSP/motor/internal
)

target_include_directories(f407_service PUBLIC
    Service/command_management
    Service/communication
    Service/line_following
    Service/motion_control
    Service/parameter_management
    Service/power_management
    Service/safety_management
    Service/state_estimation
    Service/system_monitoring
    Service/teleoperation
)
target_include_directories(f407_service PRIVATE
    Service/motion_control/internal
    Service/line_following/internal
    Service/parameter_management/internal
    Service/safety_management/internal
    Service/state_estimation/internal
    Service/system_monitoring/internal
)
target_include_directories(f407_service PRIVATE Service/communication/internal)

set(F407_APP_INCLUDE_DIRS
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    App
    App/config
    App/composition
    App/tasks
    App/debug
    App/debug/commands
    App/debug/telemetry
    App/display
    App/display/pages
    App/adapters
)
target_include_directories(f407_app PRIVATE ${F407_APP_INCLUDE_DIRS})
target_include_directories(f407_app_adapters PRIVATE ${F407_APP_INCLUDE_DIRS})

# CubeMX's freertos.c is the composition root that calls App task entrypoints.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    App/config
    App/tasks
)

target_link_libraries(f407_algorithm PUBLIC m)
target_link_libraries(f407_platform PRIVATE stm32cubemx)
target_link_libraries(f407_bsp PRIVATE f407_platform stm32cubemx)
target_link_libraries(f407_service
    PRIVATE f407_algorithm f407_bsp f407_platform
)
target_link_libraries(f407_app PRIVATE f407_service f407_platform)
target_link_libraries(f407_app_adapters PRIVATE f407_service f407_bsp f407_platform stm32cubemx)

# Release maintenance authorization is consumed while compiling f407_app.
target_compile_definitions(f407_app PRIVATE
    $<$<CONFIG:Release>:DEBUG_CONSOLE_RELEASE_REQUIRES_ARM=1>
)
target_compile_definitions(f407_app_adapters PRIVATE
    $<$<CONFIG:Release>:DEBUG_CONSOLE_RELEASE_REQUIRES_ARM=1>
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    foreach(layer_target f407_algorithm f407_platform f407_bsp f407_service f407_app f407_app_adapters)
        target_compile_options(${layer_target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endforeach()
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -Wl,--whole-archive
    f407_app
    f407_app_adapters
    -Wl,--no-whole-archive
    f407_service
    f407_algorithm
    f407_bsp
    f407_platform
    f407_algorithm
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex/.bin and printing memory usage"
)
