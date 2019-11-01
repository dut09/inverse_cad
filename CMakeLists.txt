cmake_minimum_required(VERSION 3.1)

project(inverse_cad)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions("-std=c++14 -Wall")

# Eigen.
include_directories("external/eigen-3.3.7")

include_directories("cpp")

# Main program.
add_executable(inverse_cad_demo "cpp/demo/main.cpp")