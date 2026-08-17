cmake_minimum_required(VERSION 3.20)
project(mpu6050_poll CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mpu6050 src/mpu6050.cpp)
target_include_directories(mpu6050 PUBLIC include/)

add_executable(mpu6050_poll src/main.cpp)
target_include_directories(mpu6050_poll PUBLIC include/)
target_link_libraries(mpu6050_poll PRIVATE mpu6050)
