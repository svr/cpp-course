cmake_minimum_required(VERSION 3.28)

project(drone_simulation LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/mavlink" CACHE PATH "MAVLink install prefix")

find_package(MAVLink REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

file(GLOB CONFIG_LOADER_SOURCES src/config_loaders/*.cpp)
file(GLOB SOLVER_SOURCES src/solvers/*.cpp)
file(GLOB TARGET_PROVIDER_SOURCES src/target_providers/*.cpp)
file(GLOB DRONE_STATE_SOURCES src/state/*.cpp)

add_library(mission_processor
    src/coord.cpp
    src/mission_processor.cpp
)
target_link_libraries(mission_processor PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(mission_processor PUBLIC include/ include/dto include/interfaces)

add_library(mavlink_communication
    src/mavlink_communication.cpp
)
target_link_libraries(mavlink_communication PUBLIC MAVLink::mavlink)
target_include_directories(mavlink_communication PUBLIC include/ include/dto include/interfaces)


add_library(drone_physics
    src/coord.cpp
    ${DRONE_STATE_SOURCES}
    src/drone_physics.cpp
)
target_link_libraries(drone_physics PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(drone_physics PUBLIC include/ include/dto include/interfaces)

target_link_libraries(mission_processor PUBLIC drone_physics mavlink_communication)

add_executable(drone_simulation
    ${CONFIG_LOADER_SOURCES}
    ${SOLVER_SOURCES}
    ${TARGET_PROVIDER_SOURCES}
    src/mavlink_transport_udp.cpp
    src/factory.cpp
    src/main.cpp
)
target_link_libraries(drone_simulation PRIVATE mission_processor nlohmann_json::nlohmann_json)
