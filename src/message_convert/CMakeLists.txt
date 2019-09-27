cmake_minimum_required(VERSION 2.8.3)
project(message_convert)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  geodesy
  tf
  gps_msgs
  eigen_conversions
  gps_msgs
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES message_convert
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/message_convert.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(convert_point_cloud_node src/convert_point_cloud.cpp)
target_link_libraries(convert_point_cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
 
add_executable(convert_others_node src/convert_others.cpp)
target_link_libraries(convert_others_node ${catkin_LIBRARIES} )
 
add_executable(translation_node src/translation.cpp)
target_link_libraries(translation_node ${catkin_LIBRARIES} )
 
