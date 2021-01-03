# Minimum CMake version required to generate
# our build system
cmake_minimum_required(VERSION 3.0)

# Name of our Project
project(test_rs485)

# add_executable creates an executable with
# The given name.
add_executable(test test_rs485.c)

# Make sure the executable links to the wiringPi lib
target_link_libraries(test wiringPi)
