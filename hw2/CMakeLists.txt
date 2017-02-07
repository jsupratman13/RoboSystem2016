cmake_minimum_required(VERSION 2.8.3)
project(hw2)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

