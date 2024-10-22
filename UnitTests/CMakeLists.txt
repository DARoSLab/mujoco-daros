cmake_minimum_required(VERSION 3.16)
project(UnitTest)
set(CMAKE_CXX_STANDARD 11)
# debug mode
set(CMAKE_BUILD_TYPE Debug)
add_executable(run_unit_test test.cpp)