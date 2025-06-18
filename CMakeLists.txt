cmake_minimum_required(VERSION 3.16)
project(flir_icp_calib VERSION 1.0.0)
set(FLIR_ICP_CALIB_VERSION ${PROJECT_VERSION})
set(CMAKE_CXX_STANDARD 17)

option(DEBUG_TIME_LOGGING "Enable cout for logging runtime" ON)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(DEBUG_TIME_LOGGING ON CACHE BOOL "Enable cout for logging runtime" FORCE)
else()
  set(OPTIMIZATION_FLAGS $ENV{CPP_OPTIMIZATIONS})
  set(
    CMAKE_CXX_FLAGS 
    # "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_FLAGS} -flto"
    "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -Wno-deprecated-declarations"
  )
endif()

message(STATUS "DEBUG_TIME_LOGGING is ${DEBUG_TIME_LOGGING}")

set(BATCH_SIZE 5 CACHE STRING "Fixed batch size for inference")
if(DEFINED ENV{BATCH_SIZE})
    set(BATCH_SIZE $ENV{BATCH_SIZE})
    message(STATUS "Environment variable 'BATCH_SIZE' found: Setting BATCH_SIZE to ${BATCH_SIZE}")
else()
    message(WARNING "Environment variable 'BATCH_SIZE' is not found. Using default BATCH_SIZE of ${BATCH_SIZE}")
endif()

set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cfg)
set(FLIR_TOP_CAM_SERIAL $ENV{FLIR_TOP_CAM_SERIAL})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/config.h
)
##################################################################################
# END: const header cam serial numbers
##################################################################################


link_directories(/usr/local/lib)
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()
set(OPENCV_VERSION $ENV{OPENCV_VERSION})
message(STATUS "OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(cpp_utils REQUIRED)
find_package(flirmulticamera REQUIRED)
find_package(fmt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  keiko_msgs
  geometry_msgs
  ros_node_interface
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  keiko_msgs
  geometry_msgs
  tf2_ros
  INCLUDE_DIRS
)

# ============ LIBRARY ============
add_library(${PROJECT_NAME} STATIC
  src/${PROJECT_NAME}/camera_config.cpp
  src/${PROJECT_NAME}/config_parser.cpp
  src/${PROJECT_NAME}/dataIO.cpp
  src/${PROJECT_NAME}/interface.cpp
  src/${PROJECT_NAME}/methods.cpp
  src/${PROJECT_NAME}/visualization.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC 
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT} 
  Eigen3::Eigen
  cpp_utils 
  flirmulticamera
  ${catkin_LIBRARIES}
)

# ============ EXECUTABLES ============
# ROS TRANSFORM INTERFACE

add_executable(interface_aruco src/nodes/interface_aruco.cpp)
if(DEBUG_TIME_LOGGING)
  target_compile_definitions(interface_aruco PUBLIC USE_DEBUG_TIME_LOGGING=1)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DEBUG_TIME_LOGGING=1)
endif()
target_link_libraries(interface_aruco ${PROJECT_NAME})

# Publish Result

add_executable(publish_result src/nodes/publish_result.cpp)
if(DEBUG_TIME_LOGGING)
  target_compile_definitions(publish_result PUBLIC USE_DEBUG_TIME_LOGGING=1)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DEBUG_TIME_LOGGING=1)
endif()
target_link_libraries(publish_result ${PROJECT_NAME})
