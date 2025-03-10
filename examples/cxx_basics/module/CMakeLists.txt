#set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "0e5b6991-d74f-4b3d-a41c-cf096e0b2508")
#set(CMAKE_CXX_MODULE_STD 1)
add_library(foo)
target_sources(foo PUBLIC FILE_SET CXX_MODULES FILES foo.cppm)

add_executable(cxx_module main.cpp)
target_link_libraries(cxx_module PRIVATE foo)
