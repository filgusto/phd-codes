cmake_minimum_required(VERSION 3.17)
project(DualQuaternion)

set(CMAKE_CXX_STANDARD 14)

# finding required libraries
#find_library(Quaternion
#        libQuaternion.dylib
#        PATHS /Users/filiperocha/gitSources/phd-codes/prototyping/quaternions/Quaternion/cmake-build-debug)

# generates the .so library
add_library(${PROJECT_NAME} SHARED dualquaternion.cpp dualquaternion.h main_test.cpp)

# generates the main_test executable
add_executable(main_test main_test.cpp)

# attaches libraries to main_test
target_link_libraries(main_test ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${Quaternion})