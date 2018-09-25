cmake_minimum_required(VERSION 2.8.3)
project(pick_place_server_demo_hardware_bringup)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

catkin_add_env_hooks(99.pick_place_server_demo_hardware SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
