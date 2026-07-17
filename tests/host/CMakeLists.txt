cmake_minimum_required(VERSION 3.16)

project(F407V2HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

enable_testing()

include_directories(
    ../../Platform
    ../../BSP/transport
    ../../BSP/motor
    ../../BSP/encoder/internal
    ../../BSP/imu/internal
    ../../Algorithm/kinematics
    ../../BSP/motor/internal
    fakes/config
    fakes/param
    ../../Algorithm/control
    ../../Algorithm/estimation
    ../../Algorithm/filtering
    ../../Algorithm/calibration
    ../../App/config
    fakes/chassis
    ../../Service/communication
    ../../Service/communication/internal
    ../../Service/command_management
    fakes/control
    ../../Service/parameter_management
    ../../Service/parameter_management/internal
    ../../Service/power_management
    ../../Service/safety_management
    ../../Service/safety_management/internal
    ../../Service/state_estimation
    ../../Service/system_monitoring
    ../../Service/teleoperation
    ../../Service/line_following
    ../../Service/line_following/internal
    ../../Service/motion_control
    ../../Service/motion_control/internal
)

add_library(host_platform STATIC host_platform.c)
target_include_directories(host_platform PUBLIC ../../Platform)
link_libraries(host_platform)

add_executable(test_f407_v2
    test_f407_v2.c
    ../../BSP/motor/internal/motor_hardware_layout.c
    ../../Algorithm/kinematics/differential_drive_kinematics.c
    ../../Service/system_monitoring/system_monitoring_service.c
    fakes/control/control_service.c
    ../../Service/command_management/command_management_service.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
    ../../Service/communication/internal/robot_link_protocol.c
    ../../Algorithm/estimation/wheel_speed_estimator.c
    ../../BSP/motor/internal/motor_ph_en_mapper.c
    ../../Algorithm/control/wheel_speed_pid_controller.c
)

target_include_directories(test_f407_v2 PRIVATE
    .
    ../../App/adapters
    ../../BSP
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
    ../../BSP/pid
)

add_test(NAME f407_v2_host COMMAND test_f407_v2)

add_executable(test_command_management_service
    test_command_management_service.c
    ../../Service/command_management/command_management_service.c
)
add_test(NAME command_management_service COMMAND test_command_management_service)

add_executable(test_motor_hardware_layout_2wd
    test_chassis_layout_2wd.c
    ../../BSP/motor/internal/motor_hardware_layout.c
)

target_include_directories(test_motor_hardware_layout_2wd PRIVATE
    ../../BSP
    ../../BSP/motor
)

add_test(NAME motor_hardware_layout_2wd COMMAND test_motor_hardware_layout_2wd)

add_executable(test_runtime_direction_apply test_runtime_direction_apply.c)
target_include_directories(test_runtime_direction_apply PRIVATE ../../BSP)
add_test(NAME runtime_direction_apply COMMAND test_runtime_direction_apply)

add_executable(test_straight_line_controller
    test_straight_line_controller.c
    ../../Algorithm/control/straight_line_controller.c
)
target_link_libraries(test_straight_line_controller PRIVATE m)
add_test(NAME straight_line_controller COMMAND test_straight_line_controller)

add_executable(test_esp_link_policy test_esp_link_policy.c)
target_include_directories(test_esp_link_policy PRIVATE
    ../../firmware/esp12f/F407_ESP12F)
add_test(NAME esp_link_policy COMMAND test_esp_link_policy)

add_executable(test_esp_frame_parser test_esp_frame_parser.c)
target_include_directories(test_esp_frame_parser PRIVATE
    ../../firmware/esp12f/F407_ESP12F)
add_test(NAME esp_frame_parser COMMAND test_esp_frame_parser)

add_executable(test_power_adc_driver
    test_power_adc_driver.c
    ../../BSP/adc/power_adc_driver.c
    ../../BSP/motor/internal/motor_hardware_layout.c
)

target_include_directories(test_power_adc_driver PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_power_adc_driver PRIVATE -fstack-usage)
    add_test(
        NAME power_adc_driver_stack_budget
        COMMAND ${CMAKE_COMMAND}
            -DSTACK_USAGE_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DMAX_STACK_BYTES=256
            -P ${CMAKE_CURRENT_SOURCE_DIR}/check_adc_monitor_stack.cmake
    )
endif()

add_test(NAME power_adc_driver COMMAND test_power_adc_driver)

add_executable(test_line_sensor_driver
    test_line_uart.c
    ../../BSP/line/line_sensor_driver.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
)

target_include_directories(test_line_sensor_driver PRIVATE
    .
    ../../BSP/line
    ../../BSP
    ../../BSP/motor
)

add_test(NAME line_sensor_driver COMMAND test_line_sensor_driver)

add_executable(test_line_control_service
    test_line_control_service.c
    ../../Service/line_following/internal/line_calibration_coordinator.c
    ../../Service/line_following/internal/line_parameter_sync.c
    ../../Service/line_following/line_following_service.c
    ../../Algorithm/calibration/line_sensor_calibration.c
)

target_include_directories(test_line_control_service PRIVATE
    .
    ../../App
    ../../App/config
    ../../BSP
    ../../BSP/flash
    ../../BSP/imu
    ../../BSP/line
    ../../BSP/motor
)

add_test(NAME line_control_service COMMAND test_line_control_service)

add_executable(test_line_sensor_calibration
    test_line_sensor_calibration.c
    ../../Algorithm/calibration/line_sensor_calibration.c
)

target_include_directories(test_line_sensor_calibration PRIVATE
)

add_test(NAME line_sensor_calibration COMMAND test_line_sensor_calibration)

add_executable(test_host_communication_service
    test_host_communication_service.c
    ../../Service/communication/host_communication_service.c
    ../../Service/communication/internal/remote_command_dispatcher.c
    ../../Service/communication/internal/telemetry_encoder.c
    ../../Service/communication/internal/frame_stream_parser.c
    ../../Service/communication/internal/robot_link_protocol.c
    ../../BSP/transport/host_uart_transport.c
)

target_include_directories(test_host_communication_service PRIVATE
    .
    ../../App/adapters
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/line
    ../../BSP/motor
)

add_test(NAME host_communication_service COMMAND test_host_communication_service)

add_executable(test_uart_bridge_transport
    test_uart_bridge_transport.c
    ../../BSP/transport/uart_bridge_transport.c
)
target_include_directories(test_uart_bridge_transport PRIVATE
    .
    ../../BSP/transport
)
add_test(NAME uart_bridge_transport COMMAND test_uart_bridge_transport)

add_executable(test_esp12f_boot_control
    test_esp12f_boot_control.c
    ../../BSP/esp12f/esp12f_boot_control.c
)
target_include_directories(test_esp12f_boot_control PRIVATE
    .
    ../../BSP/esp12f
)
add_test(NAME esp12f_boot_control COMMAND test_esp12f_boot_control)

add_executable(test_esp12f_flash_bridge
    test_esp12f_flash_bridge.c
    ../../App/adapters/esp12f_flash_bridge.c
)
target_include_directories(test_esp12f_flash_bridge PRIVATE
    ../../App/debug
    ../../App/adapters
    ../../BSP/esp12f
    ../../BSP/motor
    ../../BSP/transport
)
add_test(NAME esp12f_flash_bridge COMMAND test_esp12f_flash_bridge)

add_executable(test_framed_stream_parser
    test_framed_stream_parser.c
    ../../Service/communication/internal/frame_stream_parser.c
    ../../Service/communication/internal/robot_link_protocol.c
)
add_test(NAME framed_stream_parser COMMAND test_framed_stream_parser)

add_executable(test_remote_command_dispatcher
    test_remote_command_dispatcher.c
    ../../Service/communication/internal/remote_command_dispatcher.c
    ../../Service/communication/internal/robot_link_protocol.c
)
add_test(NAME remote_command_dispatcher COMMAND test_remote_command_dispatcher)

add_executable(test_telemetry_encoder
    test_telemetry_encoder.c
    ../../Service/communication/internal/telemetry_encoder.c
    ../../Service/communication/internal/robot_link_protocol.c
)
add_test(NAME telemetry_encoder COMMAND test_telemetry_encoder)

add_executable(test_motor_driver_gpio
    test_motor_driver_gpio.c
    ../../BSP/motor/internal/motor_hardware_layout.c
    ../../BSP/motor/motor_driver.c
    ../../BSP/motor/motor_break.c
    ../../BSP/motor/motor_hardware_map.c
    ../../BSP/motor/internal/motor_ph_en_mapper.c
    ../../BSP/motor/internal/motor_reversal_state_machine.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
)

target_include_directories(test_motor_driver_gpio PRIVATE
    .
    ../../BSP
    ../../BSP/motor
)

add_test(NAME motor_driver_gpio COMMAND test_motor_driver_gpio)

add_executable(test_motor_reversal_state_machine
    test_motor_reversal_state.c
    ../../BSP/motor/internal/motor_reversal_state_machine.c
)
add_test(NAME motor_reversal_state_machine COMMAND test_motor_reversal_state_machine)

add_executable(test_safety_service
    test_safety_service.c
    ../../Service/safety_management/internal/battery_voltage_monitor.c
    ../../Service/safety_management/internal/fault_stop_policy.c
    ../../Service/safety_management/safety_management_service.c
    ../../Service/system_monitoring/system_monitoring_service.c
)

target_include_directories(test_safety_service PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/motor
)

add_test(NAME safety_service COMMAND test_safety_service)

add_executable(test_safety_service_fault_enabled
    test_safety_service.c
    ../../Service/safety_management/internal/battery_voltage_monitor.c
    ../../Service/safety_management/internal/fault_stop_policy.c
    ../../Service/safety_management/safety_management_service.c
    ../../Service/system_monitoring/system_monitoring_service.c
)

target_include_directories(test_safety_service_fault_enabled PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/motor
)

target_compile_definitions(test_safety_service_fault_enabled PRIVATE
    MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=1U
)

add_test(NAME safety_service_fault_enabled COMMAND test_safety_service_fault_enabled)

add_executable(test_current_guard
    test_current_guard.c
    ../../Service/motion_control/internal/motor_current_limiter.c
    ../../BSP/motor/internal/motor_hardware_layout.c
)

target_include_directories(test_current_guard PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)

add_test(NAME current_guard COMMAND test_current_guard)

add_executable(test_current_guard_soft_limit
    test_current_guard.c
    ../../Service/motion_control/internal/motor_current_limiter.c
    ../../BSP/motor/internal/motor_hardware_layout.c
)

target_include_directories(test_current_guard_soft_limit PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)

target_compile_definitions(test_current_guard_soft_limit PRIVATE
    CURRENT_GUARD_TEST_SOFT_LIMIT=1
    MOTOR_CURRENT_GUARD_OBSERVE_ONLY=0U
    MOTOR_CURRENT_SOFT_LIMIT_ENABLED=1U
    MOTOR_CURRENT_LIMIT_A=1.0f
)

add_test(NAME current_guard_soft_limit COMMAND test_current_guard_soft_limit)

add_executable(test_chassis_service_current_limit
    test_chassis_service_current_limit.c
    fakes/chassis/chassis_service.c
    ../../Service/motion_control/motion_control_service.c
    ../../Service/motion_control/internal/motion_parameter_sync.c
    ../../Service/motion_control/internal/wheel_target_planner.c
    ../../Service/motion_control/internal/wheel_speed_control_loop.c
    ../../Service/motion_control/internal/wheel_feedback_monitor.c
    ../../Service/motion_control/internal/motor_output_coordinator.c
    ../../Service/motion_control/internal/motion_test_mode.c
    ../../Service/motion_control/internal/motion_status_builder.c
    ../../Service/motion_control/internal/motion_maintenance_service.c
    ../../Algorithm/kinematics/differential_drive_kinematics.c
    ../../Algorithm/control/straight_line_controller.c
    ../../Service/motion_control/internal/motor_current_limiter.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
    ../../BSP/motor/internal/motor_hardware_layout.c
    ../../Algorithm/control/wheel_speed_pid_controller.c
)

target_include_directories(test_chassis_service_current_limit PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
    ../../BSP/pid
)

target_link_libraries(test_chassis_service_current_limit PRIVATE m)

add_test(NAME chassis_service_current_limit COMMAND test_chassis_service_current_limit)

add_executable(test_chassis_target_planner
    test_chassis_target_planner.c
    ../../Service/motion_control/internal/wheel_target_planner.c
    ../../Algorithm/kinematics/differential_drive_kinematics.c
    ../../Algorithm/control/straight_line_controller.c
)
target_include_directories(test_chassis_target_planner PRIVATE
    ../../BSP/motor
)
target_link_libraries(test_chassis_target_planner PRIVATE m)
add_test(NAME chassis_target_planner COMMAND test_chassis_target_planner)

add_executable(test_chassis_speed_feedback
    test_chassis_speed_feedback.c
    ../../Service/motion_control/internal/wheel_speed_control_loop.c
    ../../Service/motion_control/internal/wheel_feedback_monitor.c
    ../../BSP/motor/internal/motor_hardware_layout.c
    ../../Algorithm/control/wheel_speed_pid_controller.c
)
target_include_directories(test_chassis_speed_feedback PRIVATE
    ../../BSP
    ../../BSP/encoder
    ../../BSP/motor
    fakes/chassis
)
target_link_libraries(test_chassis_speed_feedback PRIVATE m)
add_test(NAME chassis_speed_feedback COMMAND test_chassis_speed_feedback)

add_executable(test_chassis_maintenance_service
    test_chassis_maintenance_service.c
    fakes/chassis/chassis_maintenance_service.c
    ../../Service/motion_control/internal/motion_maintenance_service.c
)

target_include_directories(test_chassis_maintenance_service PRIVATE
    .
    ../../BSP
    ../../BSP/encoder
    ../../BSP/motor
)

add_test(NAME chassis_maintenance_service COMMAND test_chassis_maintenance_service)

add_executable(test_debug_maintenance_policy
    test_debug_maintenance_policy.c
    ../../App/debug/debug_maintenance_policy.c
)

target_include_directories(test_debug_maintenance_policy PRIVATE
    ../../App/debug
)

add_test(NAME debug_maintenance_policy COMMAND test_debug_maintenance_policy)

add_executable(test_debug_log_policy
    test_debug_log_policy.c
    ../../App/debug/debug_log_policy.c
)
target_include_directories(test_debug_log_policy PRIVATE ../../App/debug)
add_test(NAME debug_log_policy COMMAND test_debug_log_policy)

add_executable(test_debug_straight_telemetry
    test_debug_straight_telemetry.c
    ../../App/debug/debug_straight_telemetry.c
)
target_include_directories(test_debug_straight_telemetry PRIVATE
    ../../App/debug
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)
add_test(NAME debug_straight_telemetry COMMAND test_debug_straight_telemetry)

add_executable(test_relative_heading_controller
    test_relative_heading_controller.c
    ../../Algorithm/control/relative_heading_controller.c
)

target_include_directories(test_relative_heading_controller PRIVATE
)

target_link_libraries(test_relative_heading_controller PRIVATE m)

add_test(NAME relative_heading_controller COMMAND test_relative_heading_controller)

add_executable(test_ps2_control_service
    test_ps2_control_service.c
    fakes/control/ps2_control_service.c
    ../../Service/teleoperation/teleoperation_service.c
    ../../Algorithm/control/relative_heading_controller.c
)

target_include_directories(test_ps2_control_service PRIVATE
    .
    ../../BSP
    ../../BSP/imu
    ../../BSP/led
    ../../BSP/ps2
)

target_link_libraries(test_ps2_control_service PRIVATE m)

add_test(NAME ps2_control_service COMMAND test_ps2_control_service)

add_executable(test_oled_ssd1306
    test_oled_ssd1306.c
    ../../BSP/oled/ssd1306.c
)

target_include_directories(test_oled_ssd1306 PRIVATE
  .
  ../../BSP
  ../../BSP/oled
)

add_test(NAME oled_ssd1306 COMMAND test_oled_ssd1306)

add_executable(test_imu_pipeline
    test_imu_pipeline.c
    ../../Algorithm/calibration/imu_calibration_guard.c
    ../../Algorithm/calibration/imu_calibration.c
    ../../BSP/imu/bmi270_fifo_reader.c
    ../../Algorithm/estimation/attitude_estimator.c
    ../../Algorithm/filtering/imu_signal_filter.c
    ../../BSP/imu/bmi270_profile.c
    ../../Algorithm/estimation/imu_timestamp_tracker.c
)

target_include_directories(test_imu_pipeline PRIVATE
    .
    ../../BSP/imu
)

target_link_libraries(test_imu_pipeline PRIVATE m)

add_test(NAME imu_pipeline COMMAND test_imu_pipeline)

add_executable(test_imu_estimation_pipeline
    test_imu_estimation_pipeline.c
    ../../Service/state_estimation/internal/imu_estimation_pipeline.c
    ../../Service/state_estimation/internal/imu_quality_monitor.c
    ../../Algorithm/calibration/imu_calibration.c
    ../../Algorithm/estimation/attitude_estimator.c
    ../../Algorithm/estimation/imu_timestamp_tracker.c
    ../../Algorithm/filtering/imu_signal_filter.c
    ../../Service/parameter_management/parameter_imu_calibration.c
)
target_include_directories(test_imu_estimation_pipeline PRIVATE
    . ../../BSP/imu ../../Service/state_estimation/internal
)
target_link_libraries(test_imu_estimation_pipeline PRIVATE m)
add_test(NAME imu_estimation_pipeline COMMAND test_imu_estimation_pipeline)

add_executable(test_imu_quality_monitor
    test_imu_quality_monitor.c
    ../../Service/state_estimation/internal/imu_quality_monitor.c
)
target_include_directories(test_imu_quality_monitor PRIVATE . ../../BSP/imu ../../Service/state_estimation/internal)
add_test(NAME imu_quality_monitor COMMAND test_imu_quality_monitor)

add_executable(test_bmi270_device
    test_bmi270_device.c
    ../../BSP/imu/bmi270_device.c
    ../../BSP/imu/bmi270_config.c
    ../../BSP/imu/bmi270_profile.c
)
target_include_directories(test_bmi270_device PRIVATE . ../../BSP/imu)
add_test(NAME bmi270_device COMMAND test_bmi270_device)

add_executable(test_bmi270_driver_sequence
    test_bmi270_driver_sequence.c
    ../../BSP/imu/bmi270_driver.c
    ../../BSP/imu/bmi270_profile.c
)
target_include_directories(test_bmi270_driver_sequence PRIVATE . ../../BSP/imu)
add_test(NAME bmi270_driver_sequence COMMAND test_bmi270_driver_sequence)

add_executable(test_param_service
    test_param_service.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
)

target_include_directories(test_param_service PRIVATE
    .
    ../../BSP
    ../../BSP/motor
)

add_test(NAME param_service COMMAND test_param_service)

add_executable(test_app_config
    test_app_config.c
    ../../App/config/robot_config.c
    ../../Service/parameter_management/parameter_management_service.c
)

target_include_directories(test_app_config PRIVATE
    .
    ../../App
    ../../BSP
    ../../BSP/motor
    ../../Algorithm/kinematics
    ../../BSP/motor/internal
    fakes/config
    ../../Service/command_management
    ../../Service/communication
    ../../Service/line_following
    ../../Service/motion_control
    ../../Service/parameter_management
    ../../Service/power_management
    ../../Service/safety_management
    ../../Service/state_estimation
    ../../Service/state_estimation/internal
    ../../Service/system_monitoring
    ../../Service/system_monitoring/internal
    ../../Service/teleoperation
)

add_test(NAME app_config COMMAND test_app_config)

add_executable(test_foundation_services
    test_foundation_services.c
    ../../App/config/robot_config.c
    ../../Service/state_estimation/internal/imu_calibration_coordinator.c
    ../../Service/state_estimation/internal/imu_quality_monitor.c
    ../../Service/parameter_management/parameter_management_service.c
    ../../Service/parameter_management/internal/parameter_management_persistence.c
    ../../Service/parameter_management/parameter_imu_calibration.c
    ../../Service/state_estimation/state_estimation_service.c
    ../../Service/power_management/power_management_service.c
    ../../Service/system_monitoring/system_monitoring_service.c
    ../../Algorithm/calibration/imu_calibration_guard.c
)
target_include_directories(test_foundation_services PRIVATE
    .
    ../../App
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
    ../../Algorithm/calibration
    ../../Service/command_management
    ../../Service/communication
    ../../Service/line_following
    ../../Service/motion_control
    ../../Service/parameter_management
    ../../Service/parameter_management/internal
    ../../Service/power_management
    ../../Service/state_estimation
    ../../Service/state_estimation/internal
    ../../Service/system_monitoring
    ../../Service/teleoperation
)
target_link_libraries(test_foundation_services PRIVATE m)
add_test(NAME foundation_services COMMAND test_foundation_services)

add_executable(test_wheel_encoder_driver
    test_wheel_encoder_driver.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
    ../../BSP/encoder/wheel_encoder_driver.c
    ../../Algorithm/estimation/wheel_speed_estimator.c
    ../../Service/state_estimation/internal/wheel_estimation_pipeline.c
)

target_include_directories(test_wheel_encoder_driver PRIVATE
    .
    ../../BSP
    ../../BSP/encoder
    ../../BSP/motor
    ../../Service/state_estimation
    ../../Service/state_estimation/internal
)

target_link_libraries(test_wheel_encoder_driver PRIVATE m)

add_test(NAME wheel_encoder_driver COMMAND test_wheel_encoder_driver)

add_executable(test_flash_param
    test_flash_param.c
    fakes/param/param_service.c
    ../../Service/parameter_management/parameter_management_service.c
    ../../Service/parameter_management/internal/flash_parameter_image.c
    ../../Service/parameter_management/internal/flash_parameter_codec.c
    ../../Service/parameter_management/internal/flash_parameter_migration.c
    ../../Service/parameter_management/internal/flash_parameter_slot_store.c
    ../../Service/parameter_management/parameter_imu_calibration.c
)

target_include_directories(test_flash_param PRIVATE
    .
    ../../BSP
    ../../BSP/flash
    ../../BSP/imu
    ../../BSP/motor
)

target_compile_definitions(test_flash_param PRIVATE
    FLASH_PARAM_HOST_TEST=1
)

add_test(NAME flash_param
    COMMAND test_flash_param ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/flash_param_schema4_beta4.txt
)

add_executable(test_power_on_self_test
    test_power_on_self_test.c
    ../../Service/safety_management/internal/power_on_self_test.c
)

target_include_directories(test_power_on_self_test PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
)

target_compile_definitions(test_power_on_self_test PRIVATE
    POWER_ON_SELF_TEST_HOST_TEST=1
)

add_test(NAME power_on_self_test COMMAND test_power_on_self_test)

add_executable(test_oled_selfcheck
    test_oled_selfcheck.c
    ../../App/display/oled_selfcheck.c
)

target_include_directories(test_oled_selfcheck PRIVATE
    .
    ../../App/display
    ../../BSP
)

add_test(NAME oled_selfcheck COMMAND test_oled_selfcheck)

add_executable(test_oled_calibration_view
    test_oled_calibration_view.c
    ../../App/display/oled_calibration_view.c
)
target_include_directories(test_oled_calibration_view PRIVATE
    ../../App/display
    ../../Service/safety_management
)
add_test(NAME oled_calibration_view COMMAND test_oled_calibration_view)

add_executable(test_oled_ui_model
    test_oled_ui_model.c
    ../../App/display/oled_ui_model.c
    ../../App/display/oled_calibration_view.c
)
target_include_directories(test_oled_ui_model PRIVATE
    ../../App/display
    ../../Service/safety_management
)
add_test(NAME oled_ui_model COMMAND test_oled_ui_model)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(NAME analysis_scripts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_analysis_scripts.py)

add_executable(generate_upper_v2_golden
    generate_upper_v2_golden.c
    ../../Service/communication/internal/robot_link_protocol.c
)

target_include_directories(generate_upper_v2_golden PRIVATE
    ../../App/adapters
)

add_test(
    NAME upper_v2_golden
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/check_upper_v2_golden.py
        $<TARGET_FILE:generate_upper_v2_golden>
        ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/upper_v2_golden.json
)

add_executable(test_communication_publish_model
    test_communication_publish_model.c
    ../../App/composition/system_publish_snapshot_provider.c
    ../../Service/system_monitoring/internal/system_publish_snapshot.c
)
target_include_directories(test_communication_publish_model PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/line
    ../../BSP/motor
    fakes/config
    ../../App
    ../../App/composition
)
add_test(NAME communication_publish_model COMMAND test_communication_publish_model)

add_executable(test_debug_console_core
    test_debug_console_core.c
    ../../App/debug/debug_console_parser.c
    ../../App/debug/debug_console_registry.c
)
target_include_directories(test_debug_console_core PRIVATE
    ../../App/debug
)
target_link_libraries(test_debug_console_core PRIVATE m)
add_test(NAME debug_console_core COMMAND test_debug_console_core)
