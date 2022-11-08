## For full information http://wiki.ros.org/catkin/CMakeLists.txt 

#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(maze_escapers)

###########################################################################
## Find catkin packages and libraries for catkin and system dependencies ##
###########################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)

##########################################
## Setup for python modules and scripts ##
##########################################
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
# No new message declared.
add_message_files(
   FILES
   RobotVertex.msg
 )

 add_service_files(
   FILES
   RobotNames.srv
   Waypoint.srv
   Neighbors.srv
   AddToMap.srv
   ExplorerBid.srv
)

add_action_files(
   DIRECTORY action
   FILES
   vertex.action
)
 
 generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# No new parameters declared.

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    message_runtime
    actionlib_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

catkin_install_python(PROGRAMS 
  nodes/leader
  nodes/follower
  nodes/tf_broadcaster
  nodes/rand_waypoints
  nodes/shared_map
  nodes/explorer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
# No unit test performed.

#add_executable(vertex_action_server src/nodes/vertex_action_server)

#target_link_libraries(
#  vertex_action_server SHARED 
#  ${catkin_LIBRARIES}
#)

#add_dependencies(
#  vertex_action_server SHARED 
#  ${vertex_action_server_EXPORTED_TARGETS}
#)
