cmake_minimum_required(VERSION 3.16)
project(student_demo)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/includes)

add_library(student_lib STATIC tools/student.cpp)
add_executable(student_app src/main.cpp)

target_link_libraries(student_app student_lib)
