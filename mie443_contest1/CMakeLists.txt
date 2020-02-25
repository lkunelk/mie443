cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs
	nav_msgs
	tf
	message_generation
	)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime
)


include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(contest1 src/contest1.cpp)
target_link_libraries(contest1 ${catkin_LIBRARIES} ${OpenCV_LIB})

add_executable(arrays src/arrays.cpp)
target_link_libraries(arrays ${catkin_LIBRARIES})

add_executable(grid_position src/grid_position.cpp)
target_link_libraries(grid_position ${catkin_LIBRARIES})
