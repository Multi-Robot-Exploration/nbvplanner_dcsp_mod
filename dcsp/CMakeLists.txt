cmake_minimum_required(VERSION 2.8.3)
project(dcsp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  customPoint.msg
  agentViewElement.msg
  dcsp_msg.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   dcsp_srv.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

 catkin_package(
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES dcsp ${catkin_LIBRARIES}
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_generation

)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#    src/dcsp_server_1.cpp
#   src/dcsp_server_2.cpp
#   src/dcsp_server_3.cpp
# )

add_executable(dcsp_server_1 src/dcsp_server_1.cpp)
target_link_libraries(dcsp_server_1 ${catkin_LIBRARIES})
add_dependencies(dcsp_server_1 ${dcsp_EXPORTED_TARGETS})


add_executable(dcsp_server_2 src/dcsp_server_2.cpp)
target_link_libraries(dcsp_server_2 ${catkin_LIBRARIES})
add_dependencies(dcsp_server_2 ${dcsp_EXPORTED_TARGETS})

add_executable(dcsp_server_3 src/dcsp_server_3.cpp)
target_link_libraries(dcsp_server_3 ${catkin_LIBRARIES})
add_dependencies(dcsp_server_3 ${dcsp_EXPORTED_TARGETS})

add_executable(dcsp_server_1_client src/dcsp_server_1_client.cpp)
target_link_libraries(dcsp_server_1_client ${catkin_LIBRARIES})
add_dependencies(dcsp_server_1_client ${dcsp_EXPORTED_TARGETS})

add_executable(dcsp_server_2_client src/dcsp_server_2_client.cpp)
target_link_libraries(dcsp_server_2_client ${catkin_LIBRARIES})
add_dependencies(dcsp_server_2_client ${dcsp_EXPORTED_TARGETS})

add_executable(dcsp_server_3_client src/dcsp_server_3_client.cpp)
target_link_libraries(dcsp_server_3_client ${catkin_LIBRARIES})
add_dependencies(dcsp_server_3_client ${dcsp_EXPORTED_TARGETS})

