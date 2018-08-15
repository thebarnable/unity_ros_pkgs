cmake_minimum_required(VERSION 2.8.3)
project(unity_ros_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} default_robot_hw_sim
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
)

###########
## Build ##
###########

link_directories(${catkin_LIBRARY_DIRS})

include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

#add_library(${PROJECT_NAME} src/unity_ros_control_interface.cpp src/unity_joint.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_library(default_robot_hw_sim src/default_robot_hw_sim.cpp src/unity_joint.cpp)
#target_link_libraries(default_robot_hw_sim ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/unity_ros_control_interface_node.cpp src/unity_ros_control_interface.cpp src/default_robot_hw_sim.cpp src/unity_joint.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} default_robot_hw_sim
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES robot_hw_sim_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
