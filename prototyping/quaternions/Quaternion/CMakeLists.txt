cmake_minimum_required(VERSION 3.17)
project(Quaternion)
set(CMAKE_CXX_STANDARD 14)

# messaging the current CMake prefix path
message(STATUS "CMake Prefix path: ${CMAKE_PREFIX_PATH}")

# generates the .so shared libraries
add_library(${PROJECT_NAME} SHARED Quaternion.cpp Quaternion.h)

# includes the test directory
add_subdirectory(test)

