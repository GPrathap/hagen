cmake_minimum_required(VERSION 3.2)
project(ground_removal_filter)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)

SET( lidar_process_node ${SUB_SOURCES})
ADD_LIBRARY ( lidar_process_node  ${lidar_process_node} )           
target_link_libraries(lidar_process_node ${catkin_LIBRARIES}  ${GROUND_REMOVAL_FILTER_LIB})
