cmake_minimum_required(VERSION 3.10)
project(Day7Architecture)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_subdirectory(libs/core)

add_executable(robot_node src/main.cpp)
target_link_libraries(robot_node core_lib)

