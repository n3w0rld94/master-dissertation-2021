cmake_minimum_required(VERSION 3.12)
project(test_controller)
find_package(YARP COMPONENTS os sig dev REQUIRED)
add_executable(test_controller)
target_sources(test_controller PRIVATE test_controller.cpp)
target_link_libraries(
	test_controller 
	PRIVATE 
		YARP::YARP_os 
		YARP::YARP_init 
		YARP::YARP_sig 
		YARP::YARP_dev 
)
