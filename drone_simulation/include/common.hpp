#pragma once

constexpr int MAX_STEPS = 10000;

constexpr float EPSILON = 1e-6f;
constexpr float GRAVITY = 9.81f;

#define ENABLE_LOG 1

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