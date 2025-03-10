add_library(foo)
target_sources(foo PUBLIC FILE_SET CXX_MODULES FILES foo.cppm)

add_executable(cxx_module main.cpp)
target_link_libraries(cxx_module PRIVATE foo)
