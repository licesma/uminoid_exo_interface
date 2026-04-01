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


######################################################################################################
#                                         Submodules                                                 #
######################################################################################################

# Unitree SDK 
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building SDK examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/unitree_sdk2 unitree_sdk2)

# Librealsense
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building librealsense examples" FORCE)
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "Skip building librealsense graphical examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/librealsense librealsense)

# DynamixelSDK
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/DynamixelSDK/c++ dynamixel_sdk)



######################################################################################################
#                                          Packages                                                  #
######################################################################################################

find_package(yaml-cpp REQUIRED)



######################################################################################################
#                                         Libraries                                                  #
######################################################################################################
# upper_body_reader owns the exoskeleton sensor reader implementations
add_library(
  upper_body_reader OBJECT
  upper_body_reader/upper_body_reader.cpp
  upper_body_reader/arm_reader/arm_reader.cpp
  upper_body_reader/arm_reader/as5600/as5600_arm.cpp
  upper_body_reader/arm_reader/dynamixel/dynamixel_arm.cpp
  utils/dynamixel_timestamp_helper.cpp
)
target_include_directories(upper_body_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(upper_body_reader PUBLIC dynamixel_sdk)

# camera recorder
add_library(camera_recorder OBJECT camera/camera_recorder.cpp)
target_include_directories(camera_recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_recorder PUBLIC realsense2)

# g1 robot and controller
add_library(g1 OBJECT g1/g1Robot.cpp g1/g1Controller.cpp)
target_include_directories(g1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g1 PUBLIC unitree_sdk2)
target_compile_definitions(g1 PRIVATE
  G1_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/g1/model/upperBodyJointBounds.yaml"
  READER_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
)



######################################################################################################
#                                         Executables                                                  #
######################################################################################################


add_executable(teleoperate teleoperate.cpp)
target_include_directories(teleoperate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(teleoperate PRIVATE g1 upper_body_reader yaml-cpp)

target_compile_definitions(teleoperate PRIVATE
  CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
)

add_executable(camera camera.cpp)
target_include_directories(camera PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera PRIVATE camera_recorder)

add_executable(as5600_test as5600_test.cpp)
target_include_directories(as5600_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(as5600_test PRIVATE upper_body_reader yaml-cpp)
target_compile_definitions(as5600_test PRIVATE
  READER_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
)

add_executable(dynamixel_test dynamixel_test.cpp)
target_include_directories(dynamixel_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dynamixel_test PRIVATE upper_body_reader yaml-cpp)
target_compile_definitions(dynamixel_test PRIVATE
  READER_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
)

