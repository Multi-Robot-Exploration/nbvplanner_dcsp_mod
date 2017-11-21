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
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(
    ${PROJECT_NAME}
    src/dcsp.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
    dcsp_agent 
    src/dcsp.cpp
    src/dcsp_node.cpp
)

add_dependencies(dcsp_agent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
    dcsp_agent
    ${catkin_LIBRARIES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)