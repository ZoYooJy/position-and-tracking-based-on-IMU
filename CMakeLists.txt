cmake_minimum_required(VERSION 2.8)
project(imu_tracker)

find_package("/usr/include/eigen3")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(imu_tracker test.cpp)
target_link_libraries(imu_tracker ${Pangolin_LIBRARIES})

