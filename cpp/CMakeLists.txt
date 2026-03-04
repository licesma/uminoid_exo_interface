cmake_minimum_required(VERSION 3.5)
project(uminoid_exo_interface VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

# Pull in the Unitree SDK2 (sets up unitree_sdk2, ddsc, ddscxx targets)
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building SDK examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/unitree_sdk2 unitree_sdk2)

find_package(yaml-cpp REQUIRED)

# wjwwood/serial — built from source, no catkin required
set(WJWWOOD_SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/serial)
add_library(wjwwood_serial
    ${WJWWOOD_SERIAL_DIR}/src/serial.cc
    ${WJWWOOD_SERIAL_DIR}/src/impl/unix.cc
    ${WJWWOOD_SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc
)
target_include_directories(wjwwood_serial PUBLIC ${WJWWOOD_SERIAL_DIR}/include)
target_link_libraries(wjwwood_serial PUBLIC rt pthread)

# serial_reader handles low-level I/O; serial_manager owns the thread and value cache
add_library(serial_reader OBJECT joint_reader/serial_reader.cpp)
target_include_directories(serial_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/joint_reader)
target_link_libraries(serial_reader PUBLIC wjwwood_serial)

add_library(serial_manager OBJECT joint_reader/serial_manager.cpp)
target_include_directories(serial_manager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/joint_reader)
target_link_libraries(serial_manager PUBLIC serial_reader)

# bounds_loader loads joint bound YAML files
add_library(bounds_loader OBJECT utils/bounds_loader.cpp)
target_include_directories(bounds_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/utils)
target_link_libraries(bounds_loader PUBLIC yaml-cpp)


add_executable(swing_elbow swing_elbow.cpp)
target_include_directories(swing_elbow PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(swing_elbow PRIVATE unitree_sdk2 serial_reader serial_manager bounds_loader)

add_executable(teleoperate teleoperate.cpp g1/g1Robot.cpp g1/g1Controller.cpp)
target_include_directories(teleoperate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(teleoperate PRIVATE unitree_sdk2 serial_reader serial_manager bounds_loader yaml-cpp)

target_compile_definitions(teleoperate PRIVATE
  CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
  G1_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/g1/model/upperBodyJointBounds.yaml"
)
