cmake_minimum_required(VERSION 2.8.3)
project(people_detection)

find_package(catkin REQUIRED COMPONENTS
  people_msgs
  roscpp
  tf
  visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(fake_detection src/fake_detection.cpp)

add_dependencies(fake_detection people_msgs_gencpp visualization_msgs_gencpp)

target_link_libraries(fake_detection
  ${catkin_LIBRARIES}
)
