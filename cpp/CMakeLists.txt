cmake_minimum_required(VERSION 3.5)
project(uminoid_exo_interface VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

# Pull in the Unitree SDK2 (sets up unitree_sdk2, ddsc, ddscxx targets)
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building SDK examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/unitree_sdk2 unitree_sdk2)

# serial_value_reader is a separate compilation unit needed by joint_reader.hpp
add_library(serial_value_reader OBJECT joint_reader/serial_value_reader.cpp)
target_include_directories(serial_value_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/joint_reader)

add_executable(right_elbow_swing right_elbow_swing.cpp g1/g1Robot.cpp g1/g1Controller.cpp)
target_include_directories(right_elbow_swing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(right_elbow_swing PRIVATE unitree_sdk2 serial_value_reader)
