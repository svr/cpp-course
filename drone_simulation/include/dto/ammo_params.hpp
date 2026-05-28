#pragma once
#include "common.hpp"

struct AmmoParams {
    char name[MAX_NAME_LENGTH];
    float mass;
    float drag;
    float lift;
};