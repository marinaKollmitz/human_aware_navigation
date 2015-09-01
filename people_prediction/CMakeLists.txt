cmake_minimum_required(VERSION 2.8.3)
project(people_prediction)

find_package(catkin REQUIRED COMPONENTS
  people_msgs
  roscpp
  tf
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
)

add_executable(constant_velocity_prediction src/constant_velocity_prediction.cpp)

add_dependencies(constant_velocity_prediction people_msgs_gencpp visualization_msgs_gencpp)

target_link_libraries(constant_velocity_prediction
   ${catkin_LIBRARIES}
)
