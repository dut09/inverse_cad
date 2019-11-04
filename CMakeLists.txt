cmake_minimum_required(VERSION 3.1)

project(inverse_cad)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions("-std=c++14 -Wall -fPIC")

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

#python3-config --includes
execute_process(COMMAND bash -c "python3-config --includes|sed 's/-I//g'|awk '{print $1;}'" OUTPUT_VARIABLE PYTHON_INCLUDES)
message("Getting Python includes from the following directory:")
message(${PYTHON_INCLUDES})
include_directories(${PYTHON_INCLUDES})

# The core library.
file(GLOB_RECURSE CORE_HEADER "cpp/core/*.h")
file(GLOB_RECURSE CORE_CPP "cpp/core/*.cpp")
add_library(inverse_cad_core SHARED ${CORE_HEADER} ${CORE_CPP})

file(GLOB_RECURSE CORE_BINDING "cpp/core/*.cpp" "cpp/core/*.cxx")
add_library(scene SHARED ${CORE_HEADER} ${CORE_BINDING})

# Main program.
add_executable(inverse_cad_demo "cpp/demo/main.cpp")
target_link_libraries(inverse_cad_demo inverse_cad_core)
