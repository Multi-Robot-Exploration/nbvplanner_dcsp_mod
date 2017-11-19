cmake_minimum_required(VERSION 2.8.3)
project(frontier_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    visualization_msgs
    message_generation
    octomap_ros
    octomap_msgs
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  frontier_srv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
 LIBRARIES frontier_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES}
 CATKIN_DEPENDS 
    geometry_msgs
    visualization_msgs
    message_generation
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    octomap_ros
    octomap_msgs
)

## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
    ${PROJECT_NAME}
    src/freespace_frontier_extractor.cpp 
    src/freespace_frontier_representative.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
    ${PROJECT_NAME}_node
    src/freespace_frontier_detection_node.cpp 
    src/freespace_frontier_extractor.cpp 
    src/freespace_frontier_representative.cpp
)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)