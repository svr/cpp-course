cmake_minimum_required(VERSION 3.16)
project(imu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink" CACHE PATH "MAVLink install prefix")
find_package(MAVLink REQUIRED)
add_library(mavlink_communication
    src/mavlink/mavlink_communication.cpp
    src/mavlink/mavlink_transport_udp.cpp
)
target_link_libraries(mavlink_communication PUBLIC MAVLink::mavlink)
target_include_directories(mavlink_communication PUBLIC include/mavlink)


add_library(mpu6050_driver
    src/imu/mpu6050_driver.cpp
)
target_include_directories(mpu6050_driver PUBLIC include/imu)


add_library(imu
    src/imu/imu_manager.cpp
    src/imu/sensor_fusion.cpp
)
target_include_directories(imu PUBLIC include/imu include/mavlink)
target_link_libraries(imu PUBLIC mpu6050_driver mavlink_communication)

add_executable(imu_cli src/main.cpp)
target_link_libraries(imu_cli PRIVATE imu)
