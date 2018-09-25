cmake_minimum_required(VERSION 2.8.3)
project(pick_place_server_gripper_server)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED
  COMPONENTS
    tf2
    tf2_ros
    tf2_eigen
    robotiq_85_msgs
    actionlib_msgs
    gazebo_ros_link_attacher
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_action_files(DIRECTORY action FILES Grip.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
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

add_executable(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(gripper_server ${catkin_EXPORTED_TARGETS})

add_executable(gripper_client_test src/gripper_client_test.cpp)
target_link_libraries(gripper_client_test ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(gripper_client_test ${catkin_EXPORTED_TARGETS})
