cmake_minimum_required(VERSION 3.0.2)
project(mp_uav_landing_sim)

find_package(catkin REQUIRED COMPONENTS
    rospy
    gazebo_ros
    gazebo_msgs
    message_generation
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_python_setup()

add_message_files(
    FILES
        UWBRange.msg
        Intensity.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)
 
catkin_package(
    DEPENDS
        rospy
        gazebo_ros
        gazebo_msgs
        message_generation
)
