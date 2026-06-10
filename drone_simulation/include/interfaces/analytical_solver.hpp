#pragma once

#include "ballistic_solver.hpp"

class AnalyticalSolver : public IBallisticSolver {
    float calcHDistance(const AmmoParams& ammo, const float height, const float speed) const;
    Coord calcDropPosition(const Coord& dronePos, const Coord& targetPos, float hDist) const;

    public:
    ~AnalyticalSolver() override = default;
    void load() override {};
    float calcFlightTime(const Coord& dronePos, const Coord& targetPos, float attackSpeed, float accelerationPath) const override;
    Coord solve(const Coord& dronePos, const Coord& targetPos, float altitude, const AmmoParams& ammoParams, float speed) const override;
    Coord calcAimPoint(const Coord& dronePos, float dir, float altitude, const AmmoParams& ammoParams, float speed) const override;
};