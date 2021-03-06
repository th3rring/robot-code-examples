cmake_minimum_required(VERSION 2.8.3)
project(test_ur5_common_bringup)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2
    tf2_ros
    tf2_eigen
    pluginlib
    geometric_shapes
    geometry_msgs
    control_msgs
    vicon_object_server_msgs
    vicon_object_server_library
    robotiq_85_msgs
    actionlib_msgs
    gazebo_ros_link_attacher
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_action_files(DIRECTORY action FILES Grip.action)
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

add_executable(ur5_vicon_pick_demo src/ur5_vicon_pick_demo.cpp)
target_link_libraries(ur5_vicon_pick_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_vicon_pick_demo ${catkin_EXPORTED_TARGETS})

add_executable(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gripper_server ${catkin_EXPORTED_TARGETS})

add_executable(gripper_client_test src/gripper_client_test.cpp)
target_link_libraries(gripper_client_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gripper_client_test ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.test_ur5_common_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
