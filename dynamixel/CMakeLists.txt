cmake_minimum_required(VERSION 2.8.3)

add_library(dxl dynamixel.c dynamixel.h dxl_hal.c dxl_hal.h dynamixel_consts.h)

add_library(dxl_io dynamixel_io.h dynamixel_io.cc DynamixelMotor.h DynamixelMotor.cc)
target_link_libraries(dxl_io dxl)

add_executable(test test.c dynamixel_consts.h)
target_link_libraries(test dxl)
