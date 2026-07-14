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
    ../../Domain/chassis
    ../../Domain/config
    ../../Domain/control
    ../../Domain/encoder
    ../../Domain/imu
    ../../Domain/line
    ../../Domain/model
    ../../Domain/motor
    ../../Domain/protocol
    ../../Domain/safety
    ../../Service/chassis
    ../../Service/communication
    ../../Service/control
    ../../Service/diagnostics
    ../../Service/param
    ../../Service/safety
)

add_library(host_platform STATIC host_platform.c)
target_include_directories(host_platform PUBLIC ../../Platform)
link_libraries(host_platform)

add_executable(test_f407_v2
    test_f407_v2.c
    ../../Domain/chassis/chassis_layout.c
    ../../Domain/chassis/chassis_kinematics.c
    ../../Service/diagnostics/task_health_service.c
    ../../Service/control/control_service.c
    ../../Service/param/param_service.c
    ../../Domain/protocol/upper_protocol.c
    ../../Service/diagnostics/reset_reason_service.c
    ../../Domain/encoder/encoder_math.c
    ../../Domain/motor/motor_output_logic.c
    ../../Domain/control/pid_controller.c
)

target_include_directories(test_f407_v2 PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/param
    ../../App/param
    ../../App/chassis
    ../../App/protocol
    ../../App/monitor
    ../../BSP
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
    ../../BSP/pid
)

add_test(NAME f407_v2_host COMMAND test_f407_v2)

add_executable(test_chassis_layout_2wd
    test_chassis_layout_2wd.c
    ../../Domain/chassis/chassis_layout.c
)

target_include_directories(test_chassis_layout_2wd PRIVATE
    ../../App/chassis
    ../../BSP
    ../../BSP/motor
)

add_test(NAME chassis_layout_2wd COMMAND test_chassis_layout_2wd)

add_executable(test_runtime_direction_apply test_runtime_direction_apply.c)
target_include_directories(test_runtime_direction_apply PRIVATE ../../BSP)
add_test(NAME runtime_direction_apply COMMAND test_runtime_direction_apply)

add_executable(test_straight_controller
    test_straight_controller.c
    ../../Domain/control/straight_controller.c
)
target_include_directories(test_straight_controller PRIVATE ../../App/chassis)
target_link_libraries(test_straight_controller PRIVATE m)
add_test(NAME straight_controller COMMAND test_straight_controller)

add_executable(test_esp_link_policy test_esp_link_policy.c)
target_include_directories(test_esp_link_policy PRIVATE
    ../../firmware/esp12f/F407_ESP12F)
add_test(NAME esp_link_policy COMMAND test_esp_link_policy)

add_executable(test_esp_frame_parser test_esp_frame_parser.c)
target_include_directories(test_esp_frame_parser PRIVATE
    ../../firmware/esp12f/F407_ESP12F)
add_test(NAME esp_frame_parser COMMAND test_esp_frame_parser)

add_executable(test_adc_monitor
    test_adc_monitor.c
    ../../BSP/adc/adc_monitor.c
    ../../Domain/chassis/chassis_layout.c
)

target_include_directories(test_adc_monitor PRIVATE
    .
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
    ../../App/chassis
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_adc_monitor PRIVATE -fstack-usage)
    add_test(
        NAME adc_monitor_stack_budget
        COMMAND ${CMAKE_COMMAND}
            -DSTACK_USAGE_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DMAX_STACK_BYTES=256
            -P ${CMAKE_CURRENT_SOURCE_DIR}/check_adc_monitor_stack.cmake
    )
endif()

add_test(NAME adc_monitor COMMAND test_adc_monitor)

add_executable(test_line_uart
    test_line_uart.c
    ../../BSP/line/line_uart.c
    ../../Service/param/param_service.c
)

target_include_directories(test_line_uart PRIVATE
    .
    ../../BSP/line
    ../../App/param
    ../../App/chassis
    ../../BSP
    ../../BSP/motor
)

add_test(NAME line_uart COMMAND test_line_uart)

add_executable(test_line_control_service
    test_line_control_service.c
    ../../Service/control/line_control_service.c
    ../../Domain/line/line_calibration.c
)

target_include_directories(test_line_control_service PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/param
    ../../BSP
    ../../BSP/flash
    ../../BSP/imu
    ../../BSP/line
    ../../BSP/motor
)

add_test(NAME line_control_service COMMAND test_line_control_service)

add_executable(test_line_calibration
    test_line_calibration.c
    ../../Domain/line/line_calibration.c
)

target_include_directories(test_line_calibration PRIVATE
    ../../App/control
)

add_test(NAME line_calibration COMMAND test_line_calibration)

add_executable(test_upper_uart_service
    test_upper_uart_service.c
    ../../Service/communication/upper_uart_service.c
    ../../Domain/protocol/upper_protocol.c
    ../../BSP/transport/upper_uart_transport.c
)

target_include_directories(test_upper_uart_service PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/protocol
    ../../App/monitor
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/line
    ../../BSP/motor
)

add_test(NAME upper_uart_service COMMAND test_upper_uart_service)

add_executable(test_motor_driver_gpio
    test_motor_driver_gpio.c
    ../../Domain/chassis/chassis_layout.c
    ../../BSP/motor/motor_driver.c
    ../../Domain/motor/motor_output_logic.c
    ../../Service/param/param_service.c
)

target_include_directories(test_motor_driver_gpio PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/motor
)

add_test(NAME motor_driver_gpio COMMAND test_motor_driver_gpio)

add_executable(test_safety_service
    test_safety_service.c
    ../../Service/safety/safety_service.c
    ../../Service/diagnostics/task_health_service.c
)

target_include_directories(test_safety_service PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/monitor
    ../../App/param
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/motor
)

add_test(NAME safety_service COMMAND test_safety_service)

add_executable(test_safety_service_fault_enabled
    test_safety_service.c
    ../../Service/safety/safety_service.c
    ../../Service/diagnostics/task_health_service.c
)

target_include_directories(test_safety_service_fault_enabled PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/monitor
    ../../App/param
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
    ../../Service/safety/current_guard.c
    ../../Domain/chassis/chassis_layout.c
)

target_include_directories(test_current_guard PRIVATE
    .
    ../../App/chassis
    ../../App/current
    ../../App/param
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)

add_test(NAME current_guard COMMAND test_current_guard)

add_executable(test_current_guard_soft_limit
    test_current_guard.c
    ../../Service/safety/current_guard.c
    ../../Domain/chassis/chassis_layout.c
)

target_include_directories(test_current_guard_soft_limit PRIVATE
    .
    ../../App/chassis
    ../../App/current
    ../../App/param
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
    ../../Service/chassis/chassis_service.c
    ../../Domain/chassis/chassis_kinematics.c
    ../../Domain/control/straight_controller.c
    ../../Service/safety/current_guard.c
    ../../Service/param/param_service.c
    ../../Domain/chassis/chassis_layout.c
    ../../Domain/control/pid_controller.c
)

target_include_directories(test_chassis_service_current_limit PRIVATE
    .
    ../../App/chassis
    ../../App/current
    ../../App/control
    ../../App/monitor
    ../../App/param
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/imu
    ../../BSP/motor
    ../../BSP/pid
)

target_link_libraries(test_chassis_service_current_limit PRIVATE m)

add_test(NAME chassis_service_current_limit COMMAND test_chassis_service_current_limit)

add_executable(test_chassis_maintenance_service
    test_chassis_maintenance_service.c
    ../../Service/chassis/chassis_maintenance_service.c
)

target_include_directories(test_chassis_maintenance_service PRIVATE
    .
    ../../App/chassis
    ../../App/control
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
    ../../App/chassis
    ../../App/debug
    ../../BSP
    ../../BSP/adc
    ../../BSP/motor
)
add_test(NAME debug_straight_telemetry COMMAND test_debug_straight_telemetry)

add_executable(test_relative_yaw_control
    test_relative_yaw_control.c
    ../../Domain/control/relative_yaw_controller.c
)

target_include_directories(test_relative_yaw_control PRIVATE
    ../../App/control
)

target_link_libraries(test_relative_yaw_control PRIVATE m)

add_test(NAME relative_yaw_control COMMAND test_relative_yaw_control)

add_executable(test_ps2_control_service
    test_ps2_control_service.c
    ../../Service/control/ps2_control_service.c
    ../../Domain/control/relative_yaw_controller.c
)

target_include_directories(test_ps2_control_service PRIVATE
    .
    ../../App/chassis
    ../../App/control
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
  ../../App/chassis
  ../../BSP/oled
)

add_test(NAME oled_ssd1306 COMMAND test_oled_ssd1306)

add_executable(test_imu_pipeline
    test_imu_pipeline.c
    ../../Domain/imu/imu_calibration_gate.c
    ../../Domain/imu/imu_calibration.c
    ../../BSP/imu/imu_bmi270_fifo.c
    ../../BSP/imu/imu_bmi270_math.c
    ../../BSP/imu/imu_bmi270_profile.c
    ../../Domain/imu/imu_timestamp.c
)

target_include_directories(test_imu_pipeline PRIVATE
    .
    ../../App/imu
    ../../BSP/imu
)

target_link_libraries(test_imu_pipeline PRIVATE m)

add_test(NAME imu_pipeline COMMAND test_imu_pipeline)

add_executable(test_param_service
    test_param_service.c
    ../../Service/param/param_service.c
)

target_include_directories(test_param_service PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/motor
)

add_test(NAME param_service COMMAND test_param_service)

add_executable(test_encoder_driver
    test_encoder_driver.c
    ../../Service/param/param_service.c
    ../../BSP/encoder/encoder_driver.c
    ../../Domain/encoder/encoder_math.c
)

target_include_directories(test_encoder_driver PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/encoder
    ../../BSP/motor
)

target_link_libraries(test_encoder_driver PRIVATE m)

add_test(NAME encoder_driver COMMAND test_encoder_driver)

add_executable(test_flash_param
    test_flash_param.c
    ../../Service/param/param_service.c
    ../../Service/param/flash_param.c
    ../../Domain/imu/imu_calibration.c
)

target_include_directories(test_flash_param PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/flash
    ../../BSP/imu
    ../../BSP/motor
)

target_compile_definitions(test_flash_param PRIVATE
    FLASH_PARAM_HOST_TEST=1
)

add_test(NAME flash_param COMMAND test_flash_param)

add_executable(test_power_on_self_test
    test_post_service.c
    ../../Service/safety/post_service.c
)

target_include_directories(test_power_on_self_test PRIVATE
    .
    ../../App/chassis
    ../../App/monitor
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
    ../../App/chassis
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
    ../../BSP/imu
)
add_test(NAME oled_calibration_view COMMAND test_oled_calibration_view)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(NAME analysis_scripts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_analysis_scripts.py)

add_executable(generate_upper_v2_golden
    generate_upper_v2_golden.c
    ../../Domain/protocol/upper_protocol.c
)

target_include_directories(generate_upper_v2_golden PRIVATE
    ../../App/protocol
)

add_test(
    NAME upper_v2_golden
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/check_upper_v2_golden.py
        $<TARGET_FILE:generate_upper_v2_golden>
        ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/upper_v2_golden.json
)
