cmake_minimum_required(VERSION 3.30)
project(general)

include(FetchContent)
FetchContent_Declare(
  Boost
  URL      https://github.com/boostorg/boost/releases/download/boost-1.87.0/boost-1.87.0-cmake.tar.xz
  URL_HASH MD5=d55d43218e81ca3d0fc14436b7665bf1
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  EXCLUDE_FROM_ALL
)
set(BOOST_INCLUDE_LIBRARIES multi_index)
FetchContent_MakeAvailable(Boost)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_GENERATOR "Ninja")
set(CMAKE_C_COMPILER "/usr/bin/clang-19")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-19")

find_package(OpenCV REQUIRED)

add_subdirectory(examples)
add_subdirectory(third_party)
#add_subdirectory(test)
