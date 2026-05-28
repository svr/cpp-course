#pragma once

constexpr int MAX_STEPS = 10000;
constexpr int MAX_NAME_LENGTH = 32;

constexpr float EPSILON = 1e-6f;
constexpr float GRAVITY = 9.81f;


enum class DroneState {
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

#if ENABLE_LOG
#include <iostream>
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#include <iostream>
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif