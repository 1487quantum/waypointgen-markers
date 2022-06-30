cmake_minimum_required(VERSION 2.8.3)
project(waypointgen)

add_compile_options(-std=c++17)

find_package(Boost REQUIRED COMPONENTS system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
interactive_markers
roscpp
tf
roslib
actionlib
std_msgs
geometry_msgs
move_base_msgs
nav_msgs
visualization_msgs
message_generation
)

add_service_files(
  FILES
  Trigger.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

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
    INCLUDE_DIRS include
    LIBRARIES wpg_utils
    CATKIN_DEPENDS interactive_markers roscpp tf visualization_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(wpg_utils src/waypointgen_utils.cpp)
target_link_libraries(wpg_utils ${catkin_LIBRARIES})
add_dependencies(wpg_utils ${wpg_utils_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(setpoint_marker src/setpoint_marker.cpp src/setpoint_marker_node.cpp)
add_executable(setpoint_server src/setpoint_server.cpp src/setpoint_server_node.cpp)

add_dependencies(
    setpoint_marker ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
    setpoint_server ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    setpoint_marker yaml-cpp wpg_utils ${catkin_LIBRARIES} 
)


target_link_libraries(
    setpoint_server yaml-cpp wpg_utils ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(
    TARGETS wpg_utils
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# Unit testing
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_utest launch/utest.test test/utest.cpp)
    target_link_libraries(${PROJECT_NAME}_utest
    wpg_utils
    ${catkin_LIBRARIES}
    )
endif()
