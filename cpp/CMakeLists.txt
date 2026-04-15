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

# DFX Inspire Hand Service
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service dfx_inspire_service)



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
target_compile_definitions(upper_body_reader PRIVATE
  AS5600_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
  DYNAMIXEL_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
)

# camera recorder
add_library(camera_recorder OBJECT camera/camera_recorder.cpp)
target_include_directories(camera_recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_recorder PUBLIC realsense2)

# manus hand reader
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
add_library(manus_reader OBJECT manus/manus_reader.cpp manus/manus_hand.cpp)
target_include_directories(manus_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(manus_reader PUBLIC ${ZMQ_LIBRARIES})

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


add_executable(collect collect.cpp)
target_include_directories(collect PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(collect PRIVATE upper_body_reader inspire_retargeter manus_reader camera_recorder yaml-cpp)

add_executable(teleoperate teleoperate.cpp)
target_include_directories(teleoperate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(teleoperate PRIVATE g1 upper_body_reader yaml-cpp)

target_compile_definitions(teleoperate PRIVATE
  CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
)


file(GLOB TEST_SOURCES test/*.cpp)
# DDS-based tests that link against unitree_sdk2 instead of the default test deps
set(DDS_TESTS inspire_g1_test retarget_test camera_test)
foreach(test_source ${TEST_SOURCES})
  get_filename_component(test_name ${test_source} NAME_WE)
  if(${test_name} IN_LIST DDS_TESTS)
    continue()
  endif()
  add_executable(${test_name} ${test_source})
  target_include_directories(${test_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service/include)
  target_link_libraries(${test_name} PRIVATE upper_body_reader manus_reader yaml-cpp)
  target_compile_definitions(${test_name} PRIVATE
    AS5600_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
    DYNAMIXEL_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
  )
endforeach()

add_executable(retarget_test test/retarget_test.cpp)
target_include_directories(retarget_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(retarget_test PRIVATE inspire_retargeter manus_reader)

add_executable(camera_test test/camera_test.cpp)
target_include_directories(camera_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_test PRIVATE camera_recorder)


# DDS-based inspire hand test (communicates via G1 DDS topics)
add_executable(inspire_g1_test test/inspire_g1_test.cpp)
target_link_libraries(inspire_g1_test PRIVATE unitree_sdk2)

add_library(inspire_retargeter OBJECT hand_retarget/inspire/inspire_retargeter.cpp)
target_include_directories(inspire_retargeter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service/include)
target_link_libraries(inspire_retargeter PUBLIC yaml-cpp)



