cmake_minimum_required(VERSION 3.10)
project(control_app)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(controller control.cpp)

target_link_libraries(controller PUBLIC -lrt)


