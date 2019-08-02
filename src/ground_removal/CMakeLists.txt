cmake_minimum_required(VERSION 3.2)
project(ground_removal_filter)

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( ground_removal_comp ${SUB_SOURCES})

ADD_LIBRARY ( ground_removal_comp  ${ground_removal_comp} )
target_link_libraries(ground_removal_comp ${OpenCV_LIBS})