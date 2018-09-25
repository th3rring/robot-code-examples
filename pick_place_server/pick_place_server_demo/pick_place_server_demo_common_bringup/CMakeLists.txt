cmake_minimum_required(VERSION 2.8.3)
project(pick_place_server_demo_common_bringup)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED
  COMPONENTS
    pick_place_server_client
)

catkin_package(
  CATKIN_DEPENDS
    pick_place_server_client
)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(block_stack_demo src/block_stack_demo.cpp)
target_link_libraries(block_stack_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(block_stack_demo ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.pick_place_server_demo_common SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
