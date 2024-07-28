cmake_minimum_required(VERSION 3.10)
project(control_app)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Eigen3 3.3 REQUIRED)

add_executable(controller control.cpp actuator_control.cpp manipulator_control.cpp)

# target_include_directories(robot_control PRIVATE
#     ${CMAKE_SOURCE_DIR} # This includes the current source directory
#     ${EIGEN3_INCLUDE_DIR} # This includes the Eigen3 header files
# )

target_link_libraries(controller PUBLIC -lrt)


