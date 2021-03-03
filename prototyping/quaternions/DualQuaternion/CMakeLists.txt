cmake_minimum_required(VERSION 3.17)
project(DualQuaternion)
set(CMAKE_CXX_STANDARD 14)

# adding custom paths to find external libraries
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/Users/filiperocha/gitSources/phd-codes/prototyping/quaternions/Quaternion/)

# messaging the current CMake prefix path
message("[DEBUG_FIL] CMake Prefix path: ${CMAKE_PREFIX_PATH}")


# Quaternions lib: adiciona os headers e a biblioteca dinamica de
find_library(QUATERNION_LIB # biblioteca
        NAMES Quaternion
        HINTS "${CMAKE_PREFIX_PATH}/cmake-build-debug")
message("[DEBUG_FIL] QUATERNION_LIB = ${QUATERNION_LIB}")

find_path(QUATERNION_INCLUDE_DIR Quaternion.h)
message("[DEBUG_FIL] QUATERNION_INCLUDE_DIR = ${QUATERNION_INCLUDE_DIR}")

# generates the .so shared libraries
add_library(${PROJECT_NAME} SHARED DualQuaternion.cpp DualQuaternion.h) # diretorio


# links libraries to the project
target_link_libraries(${PROJECT_NAME} PRIVATE ${QUATERNION_LIB})
target_include_directories(${PROJECT_NAME} PRIVATE ${QUATERNION_INCLUDE_DIR})

# adding the test subdirectory
add_subdirectory(test)
