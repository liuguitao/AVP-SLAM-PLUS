cmake_minimum_required(VERSION 2.8.3)
project(simulate_robot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)