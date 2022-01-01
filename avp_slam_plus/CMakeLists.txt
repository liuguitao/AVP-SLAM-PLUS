cmake_minimum_required(VERSION 2.8.3)
project(avp_slam_plus)

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

add_executable(pointCloudFromRGBD src/pointCloudFromRGBD.cpp)
target_link_libraries(pointCloudFromRGBD ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pointCloudFromRGB src/pointCloudFromRGB.cpp)
target_link_libraries(pointCloudFromRGB ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})