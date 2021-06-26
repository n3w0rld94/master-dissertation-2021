cmake_minimum_required(VERSION 3.12)
project(ice_controller)
find_package(YARP COMPONENTS os rosmsg REQUIRED)
add_executable(ice_controller)
target_sources(ice_controller PRIVATE ice_controller.cpp)
target_link_libraries(
	ice_controller 
	PRIVATE 
		YARP::YARP_os 
		YARP::YARP_init 
		YARP::YARP_rosmsg
)
