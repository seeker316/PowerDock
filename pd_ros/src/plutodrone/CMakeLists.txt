cmake_minimum_required(VERSION 2.8.3)
project(plutodrone)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -g -O4 -march=native -Wfatal-errors")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  

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

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PlutoMsg.msg
  PlutoMsgAP.msg
  Drone_stats.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
    PlutoPilot.srv
    aruco.srv
    SetPos.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
  # actionlib_msgs
   # Or other packages containing msgs
 )

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

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include
  LIBRARIES plutodrone

 CATKIN_DEPENDS roscpp
 rospy
 std_msgs


 message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/include/opencv4)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/whyconclient.cpp
# )

add_library(plutodrone src/Communication.cpp src/Protocol.cpp)
target_link_libraries(plutodrone)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/whyconclient_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_whyconclient.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(OpenCV REQUIRED)

# find_package(realsense2 REQUIRED)


## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
#catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/apriltag  # Ensure AprilTag is properly included
)

add_executable(plutonode src/PlutoNode.cpp)
target_link_libraries(plutonode ${catkin_LIBRARIES} plutodrone)
add_dependencies(plutonode plutodrone_generate_messages_cpp)

add_executable(plutoswarm src/PlutoSwarm.cpp)
target_link_libraries(plutoswarm ${catkin_LIBRARIES} plutodrone)
#add_dependencies(plutoswarm plutodrone_generate_messages_cpp)

add_executable(key_command src/key_command.cpp)
target_link_libraries(key_command ${catkin_LIBRARIES} plutodrone)
add_dependencies(key_command plutodrone_generate_messages_cpp)

add_executable(key_handling src/key_handling.cpp)
target_link_libraries(key_handling ${catkin_LIBRARIES} plutodrone)
add_dependencies(key_handling plutodrone_generate_messages_cpp)

# add_executable(aruco src/aruco.cpp)
# target_link_libraries(aruco ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(aruco ${catkin_LIBRARIES} )

# add_dependencies(aruco plutodrone_generate_messages_cpp)
add_executable(april src/april.cpp)


target_link_libraries(april
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  apriltag
  Eigen3::Eigen
)
target_link_libraries(april ${catkin_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
    ${image_transport_LIBRARIES}
)

add_executable(pdclient src/pdclient.cpp)


target_link_libraries(pdclient
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  apriltag
  Eigen3::Eigen
)
target_link_libraries(pdclient ${catkin_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
    ${image_transport_LIBRARIES}
)

##################################################
add_executable(pdserver src/pdserver.cpp)


target_link_libraries(pdserver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  apriltag
  Eigen3::Eigen
)
target_link_libraries(pdserver ${catkin_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
    ${image_transport_LIBRARIES}
)
###################################################

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)


add_executable(cam_pub src/cam_pub.cpp)


target_link_libraries(cam_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}

)
# target_link_libraries(cam_pub ${catkin_LIBRARIES} realsense2)

add_executable(cam_sub src/cam_sub.cpp)


target_link_libraries(cam_sub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}

)

include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)

add_executable(localization src/localization.cpp)


target_link_libraries(localization
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
