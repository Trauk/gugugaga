cmake_minimum_required(VERSION 3.16)
project(opencv_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(opencv_app main.cpp)
target_link_libraries(opencv_app ${OpenCV_LIBS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/armor2.jpg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
