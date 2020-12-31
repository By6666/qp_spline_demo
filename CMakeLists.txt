cmake_minimum_required(VERSION 3.4.1)

project(qp_spline)

set(OUTPUT_NAME "exe.o")

set(CMAKE_CXX_COMPILER "g++")

add_definitions(-std=c++11)

include_directories(${CMAKE_SOURCE_DIR})
link_directories(/usr/local/qpOASES/libs)

add_subdirectory(common)
add_subdirectory(math_spline)
add_subdirectory(qp_spline_speed)


add_executable(${OUTPUT_NAME} main.cc)
target_include_directories(${OUTPUT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(${OUTPUT_NAME} common math_spline qp_spline_speed)
