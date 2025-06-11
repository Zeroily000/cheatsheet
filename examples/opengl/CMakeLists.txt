find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE glad glfw OpenGL::GL)
