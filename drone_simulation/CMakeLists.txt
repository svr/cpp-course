cmake_minimum_required(VERSION 3.28)

project(drone_simulation LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

add_executable(drone_simulation src/main.cpp)

target_link_libraries(drone_simulation PRIVATE nlohmann_json::nlohmann_json)
