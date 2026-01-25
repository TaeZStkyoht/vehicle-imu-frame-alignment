cmake_minimum_required(VERSION 3.12)

project(vehicle-imu-frame-alignment)

if(BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
