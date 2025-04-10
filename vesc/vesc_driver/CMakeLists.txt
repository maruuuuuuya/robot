cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
  vesc_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vesc_driver
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs vesc_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# driver libraries
add_library(vesc_driver
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)

# node executable
add_executable(vesc_driver_node src/vesc_driver_node.cpp)
add_dependencies(vesc_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_driver_node
  vesc_driver
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(vesc_driver_nodelet src/vesc_driver_nodelet.cpp)
add_dependencies(vesc_driver_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_driver_nodelet
  vesc_driver
  ${catkin_LIBRARIES}
)

add_dependencies(vesc_driver_node ${PROJECT_NAME}_generate_messages_cpp) 

#############
## Install ##
#############

install(TARGETS vesc_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS vesc_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS vesc_driver_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES vesc_driver_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
