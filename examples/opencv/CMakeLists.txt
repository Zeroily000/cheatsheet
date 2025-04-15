add_executable(cpu_rendering cpu_rendering.cpp)
target_include_directories(cpu_rendering PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cpu_rendering PRIVATE ${OpenCV_LIBS})
file(COPY ${PROJECT_SOURCE_DIR}/data/lena.png DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)

add_executable(gpu_rendering gpu_rendering.cpp)
target_include_directories(gpu_rendering PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gpu_rendering PRIVATE ${OpenCV_LIBS})
