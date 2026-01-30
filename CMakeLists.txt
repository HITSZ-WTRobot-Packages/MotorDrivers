cmake_minimum_required(VERSION 3.16)
project(BasicComponents LANGUAGES C CXX)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(core)
add_subdirectory(controllers)
add_subdirectory(motors)