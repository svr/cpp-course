cmake_minimum_required(VERSION 3.28)

project(drone_simulation LANGUAGES CXX)
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

FetchContent_Declare(
    cpp-httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.51.0
)

FetchContent_MakeAvailable(cpp-httplib)

find_package(Threads REQUIRED)

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

add_library(drone_physics
    src/coord.cpp
    ${DRONE_STATE_SOURCES}
    src/drone_physics.cpp
)
target_link_libraries(drone_physics PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(drone_physics PUBLIC include/ include/dto include/interfaces)

target_link_libraries(mission_processor PUBLIC drone_physics)

add_executable(drone_simulation
    ${CONFIG_LOADER_SOURCES}
    ${SOLVER_SOURCES}
    ${TARGET_PROVIDER_SOURCES}
    src/factory.cpp
    src/main.cpp
)
target_link_libraries(drone_simulation PRIVATE mission_processor nlohmann_json::nlohmann_json)

add_executable(uploader
    src/uploader_main.cpp
    src/result_uploader.cpp
)
target_include_directories(uploader PUBLIC include/ include/dto)
target_link_libraries(uploader PRIVATE httplib nlohmann_json::nlohmann_json Threads::Threads)