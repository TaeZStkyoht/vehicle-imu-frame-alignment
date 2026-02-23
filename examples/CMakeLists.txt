set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "Build Type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wconversion \
					-Wold-style-cast \
					-Wswitch-default \
					-Wduplicated-cond \
					-Wnull-dereference \
					-Wno-unknown-pragmas \
")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

if (BUILD_RUN_TIME_EXAMPLES)
	add_subdirectory(run-time)
endif()

if(BUILD_COMPILE_TIME_EXAMPLES)
	add_subdirectory(compile-time)
endif()
