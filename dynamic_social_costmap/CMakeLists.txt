cmake_minimum_required(VERSION 2.8.3)
project(dynamic_social_costmap)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    dynamic_reconfigure
    lattice_planner
    message_generation
    people_msgs
    roscpp
    rospy
    pcl_ros
)

generate_dynamic_reconfigure_options(
  cfg/SocialCostmap.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_social_costmap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    src/dynamic_social_layers.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} people_msgs_gencpp ${PROJECT_NAME}_gencfg)
