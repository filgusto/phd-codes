cmake_minimum_required(VERSION 3.17)
project(DualQuaternion)

set(CMAKE_CXX_STANDARD 14)

# generates the .so library
add_library(DualQuaternion SHARED dualquaternion.cpp dualquaternion.h main_test.cpp)

# generates the main_test executable
add_executable(main_test main_test.cpp)

# attaches libraries to main_test
target_link_libraries(main_test DualQuaternion)