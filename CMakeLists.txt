cmake_minimum_required(VERSION 2.8.3)
project(vss_sdk_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Qt5 REQUIRED COMPONENTS QtCoreConfig QtGui)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
# find_package(sqlite3 REQUIRED)

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
   c_robot_command.msg
   d_path.msg
   d_pose.msg
   global_commands.msg
   global_debug.msg
   global_state.msg
   s_ball_state.msg
   s_pose.msg
   s_rgb.msg
   s_robot_state.msg
 )

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
generate_messages(
  DEPENDENCIES
  std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES vss_sdk
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${Qt5QtCore_DIR}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  # ${SQLITE3_INCLUDE_DIRS}
  src/Interface
  src/Simulator
  src/Simulator/bullet
  src/Simulator/utils
)

add_subdirectory(
  src/Simulator/bullet/BulletCollision
)

add_subdirectory(
  src/Simulator/bullet/BulletDynamics
)

add_subdirectory(
  src/Simulator/bullet/LinearMath
)



# link_directories(${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vss_sdk.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vss_sdk_node.cpp)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vss_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

##################
# Add Executable #
##################

# include_directories(include ${catkin_INCLUDE_DIRS})

#interface
add_library(Interface src/Interface/interface.cpp)
target_link_libraries(Interface ${catkin_LIBRARIES})
add_dependencies(Interface ${PROJECT_NAME}_generate_messages_cpp)

# Viewer
add_library(Graphics src/Viewer/graphics.cpp)
target_link_libraries(Graphics ${catkin_LIBRARIES})
target_link_libraries(Graphics ${OPENGL_LIBRARIES})
target_link_libraries(Graphics ${GLUT_LIBRARIES})
add_dependencies(Graphics ${PROJECT_NAME}_generate_messages_cpp)

add_executable(viewer src/Viewer/main.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES})
target_link_libraries(viewer ${Boost_LIBRARIES})
target_link_libraries(viewer Graphics)

# Sample
add_library(Common src/Sample/common.cpp)
target_link_libraries(Common ${catkin_LIBRARIES})
add_dependencies(Common ${PROJECT_NAME}_generate_messages_cpp)

add_library(Sample_lib src/Sample/sample.cpp)
target_link_libraries(Sample_lib ${catkin_LIBRARIES})
add_dependencies(Sample_lib ${PROJECT_NAME}_generate_messages_cpp)

add_library(Strategy_sample src/Sample/strategy.cpp)
target_link_libraries(Strategy_sample Sample_lib)
target_link_libraries(Strategy_sample Common)

add_executable(sample src/Sample/main.cpp)
target_link_libraries(sample Common)
target_link_libraries(sample ${Boost_LIBRARIES})
target_link_libraries(sample Strategy_sample)
target_link_libraries(sample ${catkin_LIBRARIES})

# simulator
add_library(RobotPhysics src/Simulator/RobotPhysics.cpp)
target_link_libraries(RobotPhysics BulletCollision)
target_link_libraries(RobotPhysics BulletDynamics)
target_link_libraries(RobotPhysics LinearMath)

add_library(RobotStrategy src/Simulator/RobotStrategy.cpp)

add_library(GLDebugDrawer src/Simulator/utils/GLDebugDrawer.cpp)
target_link_libraries(GLDebugDrawer ${OPENGL_LIBRARIES})
target_link_libraries(GLDebugDrawer ${GLUT_LIBRARIES})

add_library(ModelStrategy src/Simulator/strategies/ModelStrategy.cpp)
target_link_libraries(ModelStrategy RobotStrategy)

add_library(Strategy src/Simulator/strategies/Strategy.cpp)
target_link_libraries(Strategy ModelStrategy)

add_library(StrategyBasic src/Simulator/strategies/StrategyBasic.cpp)
target_link_libraries(StrategyBasic ModelStrategy)

add_library(Physics src/Simulator/Physics.cpp)
target_link_libraries(Physics RobotStrategy)
target_link_libraries(Physics GLDebugDrawer)
target_link_libraries(Physics RobotPhysics)

add_library(Arbiter src/Simulator/Arbiter.cpp)
target_link_libraries(Arbiter Physics)

add_library(Simulator_lib src/Simulator/Simulator.cpp)
target_link_libraries(Simulator_lib ${catkin_LIBRARIES})
target_link_libraries(Simulator_lib Physics)
target_link_libraries(Simulator_lib RobotStrategy)
target_link_libraries(Simulator_lib RobotPhysics)
target_link_libraries(Simulator_lib Strategy)
target_link_libraries(Simulator_lib ModelStrategy)
target_link_libraries(Simulator_lib Interface)
target_link_libraries(Simulator_lib Arbiter)
add_dependencies(Simulator_lib ${PROJECT_NAME}_generate_messages_cpp)

add_executable(simulator src/Simulator/main.cpp)
target_link_libraries(simulator ${Boost_LIBRARIES})
target_link_libraries(simulator ${catkin_LIBRARIES})
target_link_libraries(simulator Simulator_lib)
target_link_libraries(simulator ModelStrategy)
target_link_libraries(simulator Strategy)
target_link_libraries(simulator StrategyBasic)


# Testando mensagens

add_executable(state_talker src/state_tester/talker.cpp)
target_link_libraries(state_talker ${catkin_LIBRARIES})
add_dependencies(state_talker ${PROJECT_NAME}_generate_messages_cpp)

add_executable(state_listener src/state_tester/listener.cpp)
target_link_libraries(state_listener ${catkin_LIBRARIES})
add_dependencies(state_listener ${PROJECT_NAME}_generate_messages_cpp)
