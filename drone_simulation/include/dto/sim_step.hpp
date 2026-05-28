#pragma once

#include "common.hpp"
#include "coord.hpp"

struct SimStep {
    Coord pos;
    float direction;
    float speed;
    int   targetIdx;
    DroneState state;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};