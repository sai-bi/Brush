cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )

set(src_list main.cpp)

add_executable(brush.out ${src_list})
target_link_libraries(brush.out ${OpenCV_LIBS})
