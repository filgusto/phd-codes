# generates the main_test executable
add_executable(main_test main_test.cpp)

# links inner library to test code
target_link_libraries(main_test ${PROJECT_NAME} ${QUATERNION_LIB})
target_include_directories(main_test PRIVATE ${QUATERNION_INCLUDE_DIR})






