cmake_minimum_required(VERSION 3.16)

list(APPEND EXTRA_COMPONENT_DIRS
    "${CMAKE_SOURCE_DIR}/drivers/HAL/motor_control"
    "${CMAKE_SOURCE_DIR}/drivers/HAL/encoder_reader"
    "${CMAKE_SOURCE_DIR}/drivers/simulink_control"
    "${CMAKE_SOURCE_DIR}/drivers/trajectory_generator"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(MotorEsp)