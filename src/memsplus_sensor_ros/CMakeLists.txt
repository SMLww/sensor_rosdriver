cmake_minimum_required(VERSION 3.0.2)
project(memsplus_sensor_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  geometry_msgs
  serial
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MagneticOffset.msg
  RFIDTag.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  serial 
  message_runtime   
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/include/glog)

add_executable(mprid1356_node src/mprid1356_node.cpp src/mprid1356_driver.cpp)
target_link_libraries(mprid1356_node ${catkin_LIBRARIES} glog lzma)
install(TARGETS mprid1356_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(mpmgs201_node src/mpmgs201_node.cpp src/mpmgs201_driver.cpp)
target_link_libraries(mpmgs201_node ${catkin_LIBRARIES} glog lzma)
install(TARGETS mpmgs201_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
