
# adds the test executable
add_executable(test_main teste.cpp)

# links Quaternion library with it
target_link_libraries(test_main quaternion)