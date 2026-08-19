#pragma once
#include <string>

#include "common.hpp"

struct AmmoParams {
    std::string name;
    float mass;
    float drag;
    float lift;
};