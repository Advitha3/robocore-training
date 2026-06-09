cmake_minimum_required(VERSION 3.10)
project(RobotCore)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_subdirectory(libs/safety)
add_subdirectory(libs/sensors)

add_executable(robot_node src/main.cpp)
target_link_libraries(robot_node safety_lib sensors_lib)