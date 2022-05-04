cmake_minimum_required(VERSION 2.8.3)
project(parse_rosbag)

set(CMAKE_SOURCE_DIR "/home/devin/catkin_ws/src")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3.2.0  REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
     DEPENDS EIGEN3 PCL 
   INCLUDE_DIRS include
)

add_executable(parse_rosbag parse_rosbag.cpp)
target_link_libraries(parse_rosbag ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# catkin_install_python(PROGRAMS scripts/Odometry.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
