cmake_minimum_required(VERSION 3.22)

include(FetchContent)
FetchContent_Declare(
  Boost
  URL      https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.xz
  URL_HASH MD5=893b5203b862eb9bbd08553e24ff146a
)
set(BOOST_INCLUDE_LIBRARIES multi_index)
FetchContent_MakeAvailable(Boost)

project(general)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

add_subdirectory(examples)
add_subdirectory(test)
