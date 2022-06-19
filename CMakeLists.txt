cmake_minimum_required(VERSION 2.8.3)
project(waypointgen)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
interactive_markers
roscpp
tf
roslib
actionlib
geometry_msgs
move_base_msgs
nav_msgs
visualization_msgs
message_generation
)

add_message_files(
FILES
wpg_stat.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
#DIRECTORY action
#   FILES wpServer.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypointgen
CATKIN_DEPENDS interactive_markers roscpp tf visualization_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(setpoint_marker src/setpoint_marker.cpp)
target_link_libraries(setpoint_marker
${catkin_LIBRARIES}  yaml-cpp
)

add_executable(setpoint_server src/setpoint_server.cpp)
target_link_libraries(setpoint_server
${catkin_LIBRARIES}  yaml-cpp
)

add_dependencies(
setpoint_marker setpoint_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
)