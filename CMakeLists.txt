cmake_minimum_required(VERSION 3.20)
project(sensor_fusion)
set(CMAKE_CXX_STANDARD 17)


set(SOURCE_FILES 
main.cpp
)

add_executable(sensor_fusion ${SOURCE_FILES})

target_compile_options(sensor_fusion PRIVATE -Wall -Wextra)