cmake_minimum_required(VERSION 3.0.2)
project(zed_open_capture_ros)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  roscpp
  sensor_msgs
)

## Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/ZedOpenCapture.cfg
)

## catkin specific configuration
catkin_package(
  LIBRARIES zed_open_capture_ros
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure roscpp sensor_msgs
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /catkin_ws/devel/include
  /usr/include/hidapi
)

message(STATUS "CMake Binary Dir: ${CMAKE_BINARY_DIR}")
message(STATUS "CMake Source Dir: ${PROJECT_SOURCE_DIR}")
message(STATUS "Catkin Include Dirs: ${catkin_INCLUDE_DIRS}")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/zed_open_capture_ros_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

## Link the executable with necessary libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  zed_open_capture
  pthread
)
