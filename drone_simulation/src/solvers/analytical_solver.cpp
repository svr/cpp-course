#include <cmath>
#include <iostream>
#include <algorithm>

#include "analytical_solver.hpp"


float AnalyticalSolver::calcHDistance(const AmmoParams& ammo, const float height, const float speed) const {
    const float g = GRAVITY;
    const float m = ammo.mass;
    const float d = ammo.drag;
    const float l = ammo.lift;


    float a = d * g * m - 2 * d * d * l * speed;
    float b = -3 * g * m * m + 3 * d * l * m * speed;
    float c = 6 * m * m * height;

    float p = -(b * b) / (3 * a * a);
    float q = 2 * std::pow(b, 3) / (27 * std::pow(a, 3)) + c / a;

    float acos_arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

    if (acos_arg < -1 || acos_arg > 1) {
        std::cerr << "Error in calculations. Height is too big (" << acos_arg << ")\n";
        acos_arg = std::clamp(acos_arg, -1.0f, 1.0f);
    }

    float phi = std::acos(acos_arg);
    float t = 2 * std::sqrt(-p / 3.0) * std::cos((phi + 4 * M_PI) / 3.0) - b / (3.0 * a);
    float h = speed * t -
        t * t * d * speed / (2 * m) + std::pow(t, 3) * (6 * d * g * l * m - 6 * d * d * (l * l - 1) * speed) / (36 * m * m) +
        std::pow(t, 4) * (-6 * d * d * g * l * (1 + l * l + std::pow(l, 4)) * m + 3 * std::pow(d, 3) * l * l * (1 + l * l) * speed + 6 * std::pow(d, 3) * std::pow(l, 4) * (1 + l * l) * speed) / (36 * std::pow((1 + l * l), 2) * std::pow(m, 3)) +
        std::pow(t, 5) * (3 * std::pow(d, 3) * g * std::pow(l, 3) * m - 3 * pow(d, 4) * l * l * (1 + l * l) * speed) / (36 * (1 + l * l) * std::pow(m, 4));

    return h;
}

Coord AnalyticalSolver::calcDropPosition(const Coord& dronePos, const Coord& targetPos, float hDist) const {
    Coord delta = targetPos - dronePos;
    float dist = length(delta);
    return targetPos - normalize(delta) * hDist;
}

float AnalyticalSolver::calcFlightTime(const Coord& dronePos, const Coord& targetPos, float attackSpeed, float accelerationPath) const {
    float a = attackSpeed * attackSpeed / (2 * accelerationPath);
    float D = distance(dronePos, targetPos);
    if (D <= accelerationPath) {
        return std::sqrt(2 * D / a);
    } else {
        float accelerationTime = (2 * accelerationPath) / attackSpeed;
        return accelerationTime + (D - accelerationPath) / attackSpeed;
    }
}


Coord AnalyticalSolver::solve(const Coord& dronePos, const Coord& targetPos, float altitude, const AmmoParams& ammoParams, float speed) const {
    const float HDist = calcHDistance(ammoParams, altitude, speed);
    return calcDropPosition(dronePos, targetPos, HDist);
}

Coord AnalyticalSolver::calcAimPoint(const Coord& dronePos, float dir, float altitude, const AmmoParams& ammoParams, float speed) const {
    const float HDist = calcHDistance(ammoParams, altitude, speed);
    return {
        dronePos.x + HDist * std::cos(dir),
        dronePos.y + HDist * std::sin(dir)
    };
}