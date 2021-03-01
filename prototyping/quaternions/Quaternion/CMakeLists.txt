cmake_minimum_required(VERSION 3.17)
project(Quaternion)

set(CMAKE_CXX_STANDARD 14)

# creates the .so shared libraries
add_library(Quaternion SHARED Quaternion.cpp Quaternion.h)

# compiles the teste executable
add_executable(teste teste.cpp)

# links the library to the teste target
target_link_libraries(teste Quaternion)
