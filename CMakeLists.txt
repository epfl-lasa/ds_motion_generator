cmake_minimum_required(VERSION 2.8.3)
project(ds_motion_generator)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mathlib
  sgaussians
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

 generate_dynamic_reconfigure_options(
  cfg/seDS_params.cfg
  cfg/SWIPE_params.cfg
  cfg/CYCLE_params.cfg
  cfg/PPOscilate_params.cfg
  cfg/LINEAR_params.cfg
)  


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include/analytic_DS include/learned_DS
 LIBRARIES ds_motion_generator
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs mathlib sgaussians dynamic_reconfigure 
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES ${INCLUDE_DIR}/analytic_DS/LinearMotionGenerator.h
            ${INCLUDE_DIR}/analytic_DS/PPOscilateMotionGenerator.h
            ${INCLUDE_DIR}/analytic_DS/CycleMotionGenerator.h
            ${INCLUDE_DIR}/analytic_DS/SwipeMotionGenerator.h
            ${INCLUDE_DIR}/learned_DS/seDSMotionGenerator.h)

## Specify locations of header files
## Your package locations should be listed before other locations
include_directories(include/analytic_DS include/learned_DS ${catkin_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)


## Declare a C++ executable
set(SRCS
    src/learned_DS/seDSMotionGenerator.cpp
    src/analytic_DS/SwipeMotionGenerator.cpp
    src/analytic_DS/CycleMotionGenerator.cpp
    src/analytic_DS/PPOscilateMotionGenerator.cpp
    src/analytic_DS/LinearMotionGenerator.cpp)

add_library(ds_motion_generator ${H_FILES} ${SRCS})
target_link_libraries(ds_motion_generator ${catkin_LIBRARIES})

#### Analytically Parametrized DS
add_executable(swipe_motionGenerator_node src/swipe_motionGenerator_node.cpp)
target_link_libraries(swipe_motionGenerator_node ds_motion_generator)

add_executable(cycle_motionGenerator_node  src/cycle_motionGenerator_node.cpp)
#target_link_libraries(cycle_motionGenerator_node ds_motion_generator)


add_executable(PPOscilate_motionGenerator_node src/PPOscilate_motionGenerator_node.cpp)
target_link_libraries(PPOscilate_motionGenerator_node ds_motion_generator)


add_executable(Linear_motionGenerator_node src/Linear_motionGenerator_node.cpp)
target_link_libraries(Linear_motionGenerator_node ds_motion_generator)

#### DS learned via se-DS
add_executable(seDS_motionGenerator_node src/seDS_motionGenerator_node.cpp)
target_link_libraries(seDS_motionGenerator_node ds_motion_generator)


### ADD Nodes that use the library
add_dependencies(seDS_motionGenerator_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg)

add_dependencies(swipe_motionGenerator_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_dependencies(cycle_motionGenerator_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_dependencies(PPOscilate_motionGenerator_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_dependencies(Linear_motionGenerator_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

target_link_libraries(swipe_motionGenerator_node ds_motion_generator ${catkin_LIBRARIES} )
target_link_libraries(cycle_motionGenerator_node ds_motion_generator ${catkin_LIBRARIES} )
target_link_libraries(PPOscilate_motionGenerator_node ds_motion_generator ${catkin_LIBRARIES} )
target_link_libraries(Linear_motionGenerator_node ds_motion_generator ${catkin_LIBRARIES} )
target_link_libraries(seDS_motionGenerator_node ds_motion_generator ${catkin_LIBRARIES} )
