cmake_minimum_required(VERSION 3.29)
project(CleanSpeed_6_0_Firmware)

set(CMAKE_CXX_STANDARD 14)

include_directories("Telemetry Core/include")

add_executable(CleanSpeed_6_0_Firmware
    "Telemetry Core/src/main.cpp")
