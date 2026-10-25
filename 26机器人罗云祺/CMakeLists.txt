cmake_minimum_required(VERSION 3.16)
project(show_img)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/includes)
add_library(student_lib STATIC tools/student.cpp)
add_executable(student_app src/main.cpp)
target_link_libraries(student_app student_lib)
