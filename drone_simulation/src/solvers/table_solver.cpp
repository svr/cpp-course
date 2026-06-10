#include <cmath>
#include <fstream>
#include <algorithm>
#include <stdexcept>

#include "table_solver.hpp"

Interp TableSolver::findInterp(float value, const std::vector<float>& axis) const {
    if (axis.empty()) return {0, 0.0f};
    if (value <= axis.front()) return {0, 0.0f};

    if (value >= axis.back()) {
        return { static_cast<int>(axis.size()) - 2 < 0 ? 0 : static_cast<int>(axis.size()) - 2, 1.0f };
    }

    auto it = std::lower_bound(axis.begin(), axis.end(), value);
    int hi = std::distance(axis.begin(), it);
    int lo = hi - 1;

    float frac = (value - axis[lo]) / (axis[hi] - axis[lo]);
    return {lo, frac};
}

BallisticResult TableSolver::lerp(const BallisticResult& a, const BallisticResult& b, float t) const {
    return {
        a.t + (b.t - a.t) * t,
        a.hDist + (b.hDist - a.hDist) * t
    };
}

size_t TableSolver::index(int iz, int iv, int im, int id, int il) const {
    return ((((size_t)iz * axisV0.size() + iv)
                            * axisM.size()  + im)
                            * axisD.size()  + id)
                            * axisL.size()  + il;
}

const BallisticResult& TableSolver::at(int iz, int iv, int im, int id, int il) const {
    return data[index(iz, iv, im, id, il)];
}

void TableSolver::load() {
    std::ifstream f(datafile);
    if (!f.is_open()) {
        throw std::runtime_error("TableSolver: Failed to open data file: " + datafile);
    }

    int nZ, nV, nM, nD, nL;
    if (!(f >> nZ >> nV >> nM >> nD >> nL)) return;

    axisZ0.resize(nZ); for (auto& v : axisZ0) f >> v;
    axisV0.resize(nV); for (auto& v : axisV0) f >> v;
    axisM.resize(nM);  for (auto& v : axisM)  f >> v;
    axisD.resize(nD);  for (auto& v : axisD)  f >> v;
    axisL.resize(nL);  for (auto& v : axisL)  f >> v;

    size_t total = (size_t)nZ * nV * nM * nD * nL;
    data.resize(total);

    for (size_t i = 0; i < total; i++) {
        f >> data[i].t >> data[i].hDist;
    }
}

Coord TableSolver::solve(const Coord& dronePos, const Coord& targetPos, float altitude,
                         const AmmoParams& ammoParams, float speed) const {
    float dx = targetPos.x - dronePos.x;
    float dy = targetPos.y - dronePos.y;
    float currentRange = std::sqrt(dx * dx + dy * dy);

    if (currentRange < EPSILON) return targetPos;

    BallisticResult result = lookup(altitude, speed, ammoParams.mass, ammoParams.drag, currentRange);

    if (std::abs(currentRange - result.hDist) < EPSILON) {
        return targetPos;
    }

    Coord impactPoint;
    impactPoint.x = dronePos.x + (dx / currentRange) * result.hDist;
    impactPoint.y = dronePos.y + (dy / currentRange) * result.hDist;

    return impactPoint;
}

float TableSolver::calcFlightTime(const Coord& dronePos, const Coord& targetPos,
                                  float attackSpeed, float accelerationPath) const {
    float dx = targetPos.x - dronePos.x;
    float dy = targetPos.y - dronePos.y;
    float range = std::sqrt(dx * dx + dy * dy);

    float dropAltitude = axisZ0.empty() ? 100.0f : axisZ0[axisZ0.size() / 2];
    float defaultMass  = axisM.empty() ? 1.0f : axisM[axisM.size() / 2];
    float defaultDrag  = axisD.empty() ? 0.3f : axisD[axisD.size() / 2];

    BallisticResult result = lookup(dropAltitude, attackSpeed, defaultMass, defaultDrag, range);
    return result.t;
}

Coord TableSolver::calcAimPoint(const Coord& dronePos, float dir, float altitude,
                               const AmmoParams& ammoParams, float speed) const {
    float maxRange = axisL.empty() ? 100.0f : axisL.back();
    BallisticResult result = lookup(altitude, speed, ammoParams.mass, ammoParams.drag, maxRange);

    Coord aimPoint;
    aimPoint.x = dronePos.x + result.hDist * std::cos(dir);
    aimPoint.y = dronePos.y + result.hDist * std::sin(dir);

    return aimPoint;
}

BallisticResult TableSolver::lookup(float Z0, float V0, float m, float d, float l) const {
    if (data.empty()) return {0.0f, 0.0f};

    Interp iz = findInterp(Z0, axisZ0);
    Interp iv = findInterp(V0, axisV0);
    Interp im = findInterp(m,  axisM);
    Interp id = findInterp(d,  axisD);
    Interp il = findInterp(l,  axisL);

    BallisticResult v[16];
    for (int a = 0; a < 2; a++)
     for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
       for (int e = 0; e < 2; e++) {
           auto& lo = at(iz.lo+a, iv.lo+b, im.lo+c, id.lo+e, il.lo);
           auto& hi = at(iz.lo+a, iv.lo+b, im.lo+c, id.lo+e, il.lo+1);
           v[a*8+b*4+c*2+e] = lerp(lo, hi, il.frac);
       }

    BallisticResult w[8];
    for (int a = 0; a < 2; a++)
     for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
       w[a*4+b*2+c] = lerp(v[a*8+b*4+c*2], v[a*8+b*4+c*2+1], id.frac);

    BallisticResult u[4];
    for (int a = 0; a < 2; a++)
     for (int b = 0; b < 2; b++)
      u[a*2+b] = lerp(w[a*4+b*2], w[a*4+b*2+1], im.frac);

    BallisticResult s[2];
    for (int a = 0; a < 2; a++)
        s[a] = lerp(u[a*2], u[a*2+1], iv.frac);

    return lerp(s[0], s[1], iz.frac);
}