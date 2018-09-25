cmake_minimum_required(VERSION 2.8.3)
project(test_fetch_common_bringup)

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
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    tf2
    tf2_ros
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(fetch_vicon_demo src/fetch_vicon_demo.cpp)
target_link_libraries(fetch_vicon_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(fetch_vicon_demo ${catkin_EXPORTED_TARGETS})

add_executable(fetch_vicon_pick_demo src/fetch_vicon_pick_demo.cpp)
target_link_libraries(fetch_vicon_pick_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(fetch_vicon_pick_demo ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.test_fetch_common_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)