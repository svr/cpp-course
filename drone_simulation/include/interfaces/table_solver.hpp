#pragma once

#include <vector>
#include <string>
#include "ballistic_solver.hpp"
#include "ballistic_result.hpp"
#include "interp.hpp"

class TableSolver : public IBallisticSolver {
    std::vector<float> axisZ0;
    std::vector<float> axisV0;
    std::vector<float> axisM;
    std::vector<float> axisD;
    std::vector<float> axisL;

    std::string datafile;
    std::vector<BallisticResult> data;

    size_t index(int iz, int iv, int im, int id, int il) const;
    const BallisticResult& at(int iz, int iv, int im, int id, int il) const;
    BallisticResult lookup(float Z0, float V0, float m, float d, float l) const;
    BallisticResult lerp(const BallisticResult& a, const BallisticResult& b, float t) const;
    Interp findInterp(float value, const std::vector<float>& axis) const;
public:
    TableSolver(const std::string& datafile): datafile{datafile} {};
    TableSolver() = default;
    virtual ~TableSolver() override = default;

    virtual void load() override;
    virtual Coord solve(const Coord& dronePos, const Coord& targetPos, float altitude, const AmmoParams& ammoParams, float speed) const override;
    virtual float calcFlightTime(const Coord& dronePos, const Coord& targetPos, float attackSpeed, float accelerationPath) const override;
    virtual Coord calcAimPoint(const Coord& dronePos, float dir, float altitude, const AmmoParams& ammoParams, float speed) const override;
};
