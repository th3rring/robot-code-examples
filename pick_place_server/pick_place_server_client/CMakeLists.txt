cmake_minimum_required(VERSION 2.8.3)
project(pick_place_server_client)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED
	COMPONENTS
	pick_place_server_server
	)

find_package(Boost REQUIRED system filesystem date_time thread)

set(LIBRARY_NAME pick_place_server_client_lib)

catkin_package(
	LIBRARIES ${LIBRARY_NAME}
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	pick_place_server_server
	)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(HEADERS include/pick_place_client.h)

set(SOURCES src/client.cpp)

add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
	${LIBRARY_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
