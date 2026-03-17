cmake_minimum_required(VERSION 3.8)

project(vehicle-imu-frame-alignment)

add_subdirectory(examples)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
