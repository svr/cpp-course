#pragma once

#include "common.hpp"
#include "coord.hpp"
#include "drone_config.hpp"

struct DroneContext {
    float direction;
    float targetDirection;
    float desiredDirection;

    float a;
    float speed;
    int   targetIdx;

    float turnStartTime;
    float turnRemaining;
    float turnDuration;
    float turnStartDirection;

    bool targetChanged;

    Coord pos;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;

    DroneConfig config;
};