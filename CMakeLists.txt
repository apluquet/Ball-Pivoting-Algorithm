cmake_minimum_required(VERSION 3.22)
project(TNP-TP2)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS       "-Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g3")

include_directories(eigen src)

add_executable(ball_pivoting
    src/main.cpp
    src/kdtree.cpp
    src/obj.cpp
    src/geometry.cpp
    src/pivot.cpp
    src/init.cpp)
