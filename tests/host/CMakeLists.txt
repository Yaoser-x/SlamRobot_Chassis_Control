cmake_minimum_required(VERSION 3.16)

project(F407V2HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(test_f407_v2
    test_f407_v2.c
    ../../App/chassis/chassis_math.c
    ../../App/chassis/chassis_task_timing.c
    ../../App/control/control_manager.c
    ../../App/protocol/upper_protocol.c
    ../../BSP/encoder/encoder_math.c
)

target_include_directories(test_f407_v2 PRIVATE
    .
    ../../App/chassis
    ../../App/control
    ../../App/protocol
    ../../BSP/encoder
)

add_test(NAME f407_v2_host COMMAND test_f407_v2)
