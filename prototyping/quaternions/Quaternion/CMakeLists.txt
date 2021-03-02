cmake_minimum_required(VERSION 3.17)
project(Quaternion)

set(CMAKE_CXX_STANDARD 14)

# creates the .so shared libraries
add_library(quaternion SHARED Quaternion.cpp Quaternion.h)

# includes the test directory
add_subdirectory(test)

