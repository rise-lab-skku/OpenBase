cmake_minimum_required(VERSION 2.8.3)
project(open_base)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  effort_controllers
  roscpp
  sensor_msgs
  nav_msgs
  tf
  message_generation)
find_package(cmake_modules REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(Eigen REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
#   Message1.msg
#   Message2.msg
  Velocity.msg
  MovementBezier.msg
  MovementGeneric.msg
  Movement.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
#   Service1.srv
#   Service2.srv
  FrameToFrame.srv
  KinematicsForward.srv
  KinematicsInverse.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf message_runtime
  DEPENDS Eigen3 orocos_kdl kdl_parser # system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_kinematics src/kinematics.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kinematics
  ${catkin_LIBRARIES}
  ${orocos-kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_kinematics open_base_generate_messages_cpp)

add_executable(${PROJECT_NAME}_odometry src/odometry.cpp)
target_link_libraries(
  ${PROJECT_NAME}_odometry
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_odometry open_base_generate_messages_cpp)

add_executable(${PROJECT_NAME}_sensor_encoder src/sensor/encoder.cpp)
target_link_libraries(
  ${PROJECT_NAME}_sensor_encoder
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_sensor_encoder open_base_generate_messages_cpp)

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(
  ${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_main open_base_generate_messages_cpp)

# ====================================
add_executable(wheel_command src/wheel_command.cpp)
target_link_libraries(
  wheel_command
  ${catkin_LIBRARIES}
)
add_dependencies(wheel_command open_base_generate_messages_cpp)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
