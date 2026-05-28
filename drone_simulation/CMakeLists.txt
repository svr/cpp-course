cmake_minimum_required(VERSION 3.28)

project(drone_simulation LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

file(GLOB CONFIG_LOADER_SOURCES src/config_loaders/*.cpp)
file(GLOB SOLVER_SOURCES src/solvers/*.cpp)
file(GLOB TARGET_PROVIDER_SOURCES src/target_providers/*.cpp)

add_library(mission_processor
    src/coord.cpp
    src/mission_processor.cpp
)

target_link_libraries(mission_processor PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(mission_processor PUBLIC include/ include/dto include/interfaces)

add_executable(drone_simulation
    ${CONFIG_LOADER_SOURCES}
    ${SOLVER_SOURCES}
    ${TARGET_PROVIDER_SOURCES}
    src/factory.cpp
    src/main.cpp
)
target_link_libraries(drone_simulation PRIVATE mission_processor nlohmann_json::nlohmann_json)
