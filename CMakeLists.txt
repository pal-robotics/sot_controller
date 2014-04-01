cmake_minimum_required(VERSION 2.8.3)
project(sot_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
  REQUIRED
  COMPONENTS
    controller_interface
    controller_manager
    dynamic_graph_bridge
    forward_command_controller
    realtime_tools
    ros_sot_robot_model
    roscpp
    rospy
    urdf
    dynamic-graph
    dynamic-graph-python
    sot-core
    sot-dyninv
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "-std=c++0x")


## Uncomment this if the package has a setup.py. This macro ensures
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
# modules and global scripts declared therein get installed
catkin_python_setup()

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sot_controller
  CATKIN_DEPENDS
    controller_interface
    controller_manager
    dynamic_graph_bridge
    forward_command_controller
    realtime_tools
    ros_sot_robot_model
    roscpp
    rospy
    urdf
    dynamic-graph
    dynamic-graph-python
    sot-core
    sot-dyninv
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(sot_controller
  src/sot_controller.cpp src/sot_device.cpp include/${PROJECT_NAME}/sot_controller.h include/${PROJECT_NAME}/sot_device.h
)

target_link_libraries(sot_controller
	${catkin_LIBRARIES} 
)

## Declare a cpp executable
add_executable(sot_controller_test test/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sot_controller_node sot_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sot_controller_test
   ${catkin_LIBRARIES} sot_controller 
)



# LAAS CMAKE CONFIGURATION
include(cmake/base.cmake)
include(cmake/python.cmake)

set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
)



############
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
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## copy launch files
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.launch"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/sot_controller.yaml
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  sot_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############

## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sot_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
