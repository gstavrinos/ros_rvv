cmake_minimum_required(VERSION 2.8.3)
project(ros_rvv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package()

include_directories()

