cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )

# find_package(OpenMP)
# if (OPENMP_FOUND)
    # set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    # set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

set(src_list brush.cpp)

add_executable(brush.out ${src_list})
target_link_libraries(brush.out ${OpenCV_LIBS})

set(src_list_1 main.cpp)
add_executable(main.out ${src_list_1})
target_link_libraries(main.out ${OpenCV_LIBS})


