cmake_minimum_required(VERSION 3.28)

project(drone_simulation LANGUAGES CXX)

find_library(GPIOD_LIB
    NAMES libgpiod.so.3 gpiod
    REQUIRED
)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

file(GLOB CONFIG_LOADER_SOURCES src/config_loaders/*.cpp)
file(GLOB SOLVER_SOURCES src/solvers/*.cpp)
file(GLOB TARGET_PROVIDER_SOURCES src/target_providers/*.cpp)
file(GLOB DRONE_STATE_SOURCES src/state/*.cpp)

add_library(includes INTERFACE)
target_include_directories(includes INTERFACE
    include/
    include/dto
    include/interfaces
)

add_library(gpio_trigger
    src/hardware/gpio.cpp
)
target_link_libraries(gpio_trigger PRIVATE includes ${GPIOD_LIB})

add_library(uart_data_provider
    src/hardware/uart.cpp
    src/hardware/uart_data_provider.cpp
)
target_link_libraries(uart_data_provider PRIVATE includes)


add_library(drone_physics
    src/coord.cpp
    ${DRONE_STATE_SOURCES}
    src/drone_physics.cpp
    src/drone_uart.cpp
)
target_link_libraries(drone_physics PUBLIC includes uart_data_provider nlohmann_json::nlohmann_json)


add_library(mission_processor
    src/coord.cpp
    src/mission_processor.cpp
)
target_link_libraries(mission_processor PUBLIC includes gpio_trigger drone_physics nlohmann_json::nlohmann_json)

add_executable(student
    ${CONFIG_LOADER_SOURCES}
    ${SOLVER_SOURCES}
    ${TARGET_PROVIDER_SOURCES}
    ${DRONE_STATE_SOURCES}
    src/coord.cpp
    src/factory.cpp
    src/parse_args.cpp
    src/main.cpp
)

target_link_libraries(student PRIVATE mission_processor)