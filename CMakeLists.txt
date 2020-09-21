cmake_minimum_required(VERSION 2.8.3)
project(dtz_demonstration)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS message_generation rospy)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        pluginlib
        geometric_shapes
        control_msgs
        franka_gripper
        control_msgs
        rosgraph_msgs
        rospy
)



find_package(Boost REQUIRED system filesystem date_time thread)


set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include

)

catkin_package()



set(SOURCES src/robot_demonstration.cpp src/PandaRobot.h src/positions.h)
add_executable(robot_demonstration  ${SOURCES})
target_link_libraries(robot_demonstration ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARY})

set(SOURCES src/print_franka_joints.cpp)
add_executable(print_franka_joints  ${SOURCES})
target_link_libraries(print_franka_joints ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARY})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY franka_ros/franka_gripper/include/franka_gripper/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.h"
  	PATTERN ".svn" EXCLUDE)


install(PROGRAMS scripts/opc_ua_ros_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS robot_demonstration DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
