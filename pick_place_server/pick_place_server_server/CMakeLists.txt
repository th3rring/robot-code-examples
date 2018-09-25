cmake_minimum_required(VERSION 2.8.3)
project(pick_place_server_server)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2
    tf2_ros
    tf2_eigen
    vicon_object_server_msgs
    vicon_object_server_library
    robotiq_85_msgs
    actionlib_msgs
    gazebo_ros_link_attacher
    pick_place_server_gripper_server
    robowflex_library
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_action_files(DIRECTORY action FILES PickPlace.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    tf2
    tf2_ros
    actionlib_msgs
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(pick_place_server_server src/pick_place_server.cpp)
target_link_libraries(pick_place_server_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pick_place_server_server ${catkin_EXPORTED_TARGETS})

add_executable(pick_place_server_client_test src/pick_place_server_client_test.cpp)
target_link_libraries(pick_place_server_client_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pick_place_server_client_test ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.pick_place_server_server_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
