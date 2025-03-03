cmake_minimum_required(VERSION 3.15)
project(general)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

add_subdirectory(examples)
add_subdirectory(test)
