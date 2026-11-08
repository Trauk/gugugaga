cmake_minimum_required(VERSION 3.16)
project(training3)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(task1 task1.cpp)
target_link_libraries(task1 PRIVATE ${OpenCV_LIBS})
