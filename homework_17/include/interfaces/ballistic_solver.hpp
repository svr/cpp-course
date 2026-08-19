#pragma once

#include "coord.hpp"
#include "ammo_params.hpp"

class IBallisticSolver {
public:
    virtual void load() = 0;
    virtual Coord solve(const Coord& dronePos, const Coord& targetPos, float altitude, const AmmoParams& ammoParams, float speed) const = 0;
    virtual float calcFlightTime(const Coord& dronePos, const Coord& targetPos, float attackSpeed, float accelerationPath) const = 0;
    virtual Coord calcAimPoint(const Coord& dronePos, float dir, float altitude, const AmmoParams& ammoParams, float speed) const = 0;
    virtual ~IBallisticSolver() = default;
};