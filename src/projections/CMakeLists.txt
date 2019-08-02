cmake_minimum_required(VERSION 3.2)
project(ground_removal_filter)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( lidar_projections ${SUB_SOURCES})

ADD_LIBRARY ( lidar_projections  ${lidar_projections} )