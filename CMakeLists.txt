cmake_minimum_required(VERSION 3.8.0)
project(ground_removal_filter LANGUAGES CXX)

add_definitions(-std=c++11)
SET(CMAKE_C_FLAGS_DEBUG "-D_DEBUG")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules")
set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
set(Boost_USE_STATIC_LIBS OFF)

find_package(catkin_simple REQUIRED)


set(Boost_NO_SYSTEM_PATHS OFF)
if (Boost_NO_SYSTEM_PATHS)
  set(BOOST_ROOT /usr/local)
  set(BOOST_INCLUDE_DIRS "${BOOST_ROOT}/include")
  set(BOOST_LIBRARY_DIRS "${BOOST_ROOT}/lib")
endif (Boost_NO_SYSTEM_PATHS)

find_package(Boost COMPONENTS program_options log log_setup regex atomic
                            date_time system chrono filesystem thread graph REQUIRED)
find_package(PCL)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${BOOST_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/libs/eigen_lib_3_3_7/)

set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} ${Boost_LIBRARIES})
set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY})
set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} ${PCL_LIBRARIES})
set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} ${catkin_LIBRARIES})
set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} ${OpenCV_LIBS})

message( STATUS "Include sub-directories into build process:" )
foreach( DIR lidar_subnode utils projections ground_removal)
    message( STATUS "  - include/" ${DIR} )
    include_directories( "src/${DIR}" )
    link_directories("src/${DIR}")
    add_subdirectory( "src/${DIR}" )
endforeach()

# Add the trajectory estimator library
set(GROUND_REMOVAL_FILTER_LIB ${GROUND_REMOVAL_FILTER_LIB} 
                      lidar_process_node hagen_utils lidar_projections ground_removal_comp  cnpy z)

add_executable(${PROJECT_NAME}_node nodes/hagen_node.cc 
                                          nodes/ground_removal_filter_node.cc)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_11)
target_link_libraries(${PROJECT_NAME}_node -Wl,--start-group  
                                ${GROUND_REMOVAL_FILTER_LIB} -Wl,--end-group)

install(
    DIRECTORY
     launch/
    DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    TARGETS
     ${PROJECT_NAME}_node
    LIBRARY DESTINATION
     ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)

