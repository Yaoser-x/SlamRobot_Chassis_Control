cmake_minimum_required(VERSION 3.16)

project(F407V2HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(test_f407_v2
    test_f407_v2.c
    ../../BSP/chassis_layout.c
    ../../App/chassis/chassis_math.c
    ../../App/chassis/chassis_task_timing.c
    ../../App/control/control_manager.c
    ../../App/protocol/upper_protocol.c
    ../../BSP/encoder/encoder_math.c
    ../../BSP/motor/motor_output_logic.c
    ../../BSP/pid/pid_controller.c
)

target_include_directories(test_f407_v2 PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/protocol
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
)

target_include_directories(test_line_uart PRIVATE
    .
    ../../BSP/line
)

add_test(NAME line_uart COMMAND test_line_uart)

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
)

target_include_directories(test_motor_driver_gpio PRIVATE
    .
    ../../App/chassis
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
    ../../App/current/current_guard.c
    ../../BSP/chassis_layout.c
    ../../BSP/pid/pid_controller.c
)

target_include_directories(test_chassis_control_current_limit PRIVATE
    .
    ../../App/chassis
    ../../App/current
    ../../App/control
    ../../BSP
    ../../BSP/adc
    ../../BSP/encoder
    ../../BSP/motor
    ../../BSP/pid
)

add_test(NAME chassis_control_current_limit COMMAND test_chassis_control_current_limit)

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
    ../../BSP/imu/imu_bmi270_calibration.c
    ../../BSP/imu/imu_bmi270_fifo.c
    ../../BSP/imu/imu_bmi270_math.c
    ../../BSP/imu/imu_bmi270_profile.c
    ../../BSP/imu/imu_bmi270_time.c
)

target_include_directories(test_imu_pipeline PRIVATE
    .
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

add_executable(test_flash_param
    test_flash_param.c
    ../../App/param/param_store.c
    ../../BSP/flash/flash_param.c
)

target_include_directories(test_flash_param PRIVATE
    .
    ../../App/chassis
    ../../App/param
    ../../BSP
    ../../BSP/flash
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
