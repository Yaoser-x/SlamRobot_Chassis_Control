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

add_test(NAME adc_monitor COMMAND test_adc_monitor)
