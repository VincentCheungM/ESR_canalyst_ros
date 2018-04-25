cmake_minimum_required(VERSION 2.8.3)
project(canalystii_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  canalystii_node_msg
  tf
  )


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs canalystii_node_msg
  INCLUDE_DIRS include
)


#add_library(canalystii src/canalystii.cpp ${catkin_LIBRARIES})
#target_link_libraries(canalystii  ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so  ${catkin_LIBRARIES})

#add_library(canalystii_node src/canalystii_node.cpp ${catkin_LIBRARIES})
#target_link_libraries(canalystii_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

#add_executable(canalystii_node_ros src/canalystii_node_ros.cpp) 
#target_link_libraries(canalystii_node_ros canalystii canalystii_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

add_executable(canalystii_node_ros src/canalystii_node_ros.cpp src/canalystii.cpp src/canalystii_node.cpp) 
add_dependencies(canalystii_node_ros ${canalystii_node_msg_EXPORTED_TARGETS})
target_link_libraries(canalystii_node_ros ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})
