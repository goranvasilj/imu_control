cmake_minimum_required(VERSION 2.8.3)
project(fly_jacket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
  message_generation
  tf2
  tf2_msgs
  tf2_ros

)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    serial
    sensor_msgs
    std_msgs
    tf2
    tf2_msgs
    tf2_ros
    
)



