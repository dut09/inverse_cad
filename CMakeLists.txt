cmake_minimum_required(VERSION 3.1)

project(inverse_cad)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions("-std=c++14 -Wall")

# CGAL.
find_package(CGAL REQUIRED)
if(CGAL_FOUND)
    include(${CGAL_USE_FILE})
    # CGAL include folder.
    set(CGAL_INCLUDE_DIR "${CGAL_DIR}/include")
    include_directories(${CGAL_INCLUDE_DIR})
endif()

# Eigen.
include_directories("external/eigen-3.3.7")

include_directories("cpp")

# The core library.
file(GLOB_RECURSE CORE_HEADER "cpp/core/*.h")
file(GLOB_RECURSE CORE_CPP "cpp/core/*.cpp")
add_library(inverse_cad_core SHARED ${CORE_HEADER} ${CORE_CPP})


# Main program.
add_executable(inverse_cad_demo "cpp/demo/main.cpp")
target_link_libraries(inverse_cad_demo inverse_cad_core)