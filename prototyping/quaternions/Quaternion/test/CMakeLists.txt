
# adds the test executable
add_executable(test_main test_main.cpp)

# links Quaternion library with it
target_link_libraries(test_main ${PROJECT_NAME})