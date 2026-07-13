cmake_minimum_required(VERSION 3.16)

project(F407V2HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

enable_testing()

add_executable(test_f407_v2
    test_f407_v2.c
    ../../BSP/chassis_layout.c
    ../../App/chassis/chassis_math.c
    ../../App/chassis/chassis_task_timing.c
    ../../App/control/control_manager.c
    ../../App/param/param_store.c
    ../../App/protocol/upper_protocol.c
    ../../App/monitor/reset_reason.c
    ../../BSP/encoder/encoder_math.c
    ../../BSP/motor/motor_output_logic.c
    ../../BSP/pid/pid_controller.c
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
    ../../BSP/chassis_layout.c
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
    ../../App/chassis/straight_controller.c
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
    ../../BSP/chassis_layout.c
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
    ../../App/param/param_store.c
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

add_executable(test_line_control
    test_line_control.c
    ../../App/control/line_control.c
    ../../App/control/line_calibration.c
)

target_include_directories(test_line_control PRIVATE
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

add_test(NAME line_control COMMAND test_line_control)

add_executable(test_line_calibration
    test_line_calibration.c
    ../../App/control/line_calibration.c
)

target_include_directories(test_line_calibration PRIVATE
    ../../App/control
)

add_test(NAME line_calibration COMMAND test_line_calibration)

add_executable(test_upper_uart
    test_upper_uart.c
    ../../App/protocol/upper_uart.c
    ../../App/protocol/upper_protocol.c
)

target_include_directories(test_upper_uart PRIVATE
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

add_test(NAME upper_uart COMMAND test_upper_uart)

add_executable(test_motor_driver_gpio
    test_motor_driver_gpio.c
    ../../BSP/chassis_layout.c
    ../../BSP/motor/motor_driver.c
    ../../BSP/motor/motor_output_logic.c
    ../../App/param/param_store.c
)

target_include_directories(test_motor_driver_gpio PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/motor
)

add_test(NAME motor_driver_gpio COMMAND test_motor_driver_gpio)

add_executable(test_system_monitor
    test_system_monitor.c
    ../../App/monitor/system_monitor.c
    ../../App/chassis/chassis_task_timing.c
)

target_include_directories(test_system_monitor PRIVATE
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

add_test(NAME system_monitor COMMAND test_system_monitor)

add_executable(test_system_monitor_fault_enabled
    test_system_monitor.c
    ../../App/monitor/system_monitor.c
    ../../App/chassis/chassis_task_timing.c
)

target_include_directories(test_system_monitor_fault_enabled PRIVATE
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

target_compile_definitions(test_system_monitor_fault_enabled PRIVATE
    MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=1U
)

add_test(NAME system_monitor_fault_enabled COMMAND test_system_monitor_fault_enabled)

add_executable(test_current_guard
    test_current_guard.c
    ../../App/current/current_guard.c
    ../../BSP/chassis_layout.c
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
    ../../App/current/current_guard.c
    ../../BSP/chassis_layout.c
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

add_executable(test_chassis_control_current_limit
    test_chassis_control_current_limit.c
    ../../App/chassis/chassis_control.c
    ../../App/chassis/chassis_math.c
    ../../App/chassis/straight_controller.c
    ../../App/current/current_guard.c
    ../../App/param/param_store.c
    ../../BSP/chassis_layout.c
    ../../BSP/pid/pid_controller.c
)

target_include_directories(test_chassis_control_current_limit PRIVATE
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

target_link_libraries(test_chassis_control_current_limit PRIVATE m)

add_test(NAME chassis_control_current_limit COMMAND test_chassis_control_current_limit)

add_executable(test_chassis_maintenance
    test_chassis_maintenance.c
    ../../App/chassis/chassis_maintenance.c
)

target_include_directories(test_chassis_maintenance PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../BSP
    ../../BSP/encoder
    ../../BSP/motor
)

add_test(NAME chassis_maintenance COMMAND test_chassis_maintenance)

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
    ../../App/control/relative_yaw_control.c
)

target_include_directories(test_relative_yaw_control PRIVATE
    ../../App/control
)

target_link_libraries(test_relative_yaw_control PRIVATE m)

add_test(NAME relative_yaw_control COMMAND test_relative_yaw_control)

add_executable(test_ps2_control
    test_ps2_control.c
    ../../App/control/ps2_control.c
    ../../App/control/relative_yaw_control.c
)

target_include_directories(test_ps2_control PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../BSP
    ../../BSP/imu
    ../../BSP/led
    ../../BSP/ps2
)

target_link_libraries(test_ps2_control PRIVATE m)

add_test(NAME ps2_control COMMAND test_ps2_control)

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
    ../../App/imu/imu_calibration_gate.c
    ../../BSP/imu/imu_bmi270_calibration.c
    ../../BSP/imu/imu_bmi270_fifo.c
    ../../BSP/imu/imu_bmi270_math.c
    ../../BSP/imu/imu_bmi270_profile.c
    ../../BSP/imu/imu_bmi270_time.c
)

target_include_directories(test_imu_pipeline PRIVATE
    .
    ../../App/imu
    ../../BSP/imu
)

target_link_libraries(test_imu_pipeline PRIVATE m)

add_test(NAME imu_pipeline COMMAND test_imu_pipeline)

add_executable(test_param_store
    test_param_store.c
    ../../App/param/param_store.c
)

target_include_directories(test_param_store PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/motor
)

add_test(NAME param_store COMMAND test_param_store)

add_executable(test_encoder_driver
    test_encoder_driver.c
    ../../App/param/param_store.c
    ../../BSP/encoder/encoder_driver.c
    ../../BSP/encoder/encoder_math.c
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
    ../../App/param/param_store.c
    ../../BSP/flash/flash_param.c
    ../../BSP/imu/imu_bmi270_calibration.c
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
    test_power_on_self_test.c
    ../../App/monitor/power_on_self_test.c
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
    ../../App/protocol/upper_protocol.c
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
