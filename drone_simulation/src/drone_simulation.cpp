#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "drone_simulation.hpp"

Coord& Coord::operator+=(const Coord& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Coord& Coord::operator-=(const Coord& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Coord& Coord::operator*=(float s) {
    x *= s;
    y *= s;
    return *this;
}

Coord& Coord::operator/=(float s) {
    if (std::abs(s) < EPSILON) {
        return *this;
    }
    x /= s;
    y /= s;
    return *this;
}

Coord Coord::operator+(const Coord& other) const {
    Coord result(*this);
    result += other;
    return result;
}

Coord Coord::operator-(const Coord& other) const {
    Coord result(*this);
    result -= other;
    return result;
}

Coord Coord::operator*(float s) const {
    Coord result(*this);
    result *= s;
    return result;
}

Coord Coord::operator/(float s) const {
    Coord result(*this);
    result /= s;
    return result;
}

bool Coord::operator==(const Coord& other) const {
    return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
}

float length(const Coord& coord) {
    return std::hypot(coord.x, coord.y);
}

float distance(const Coord& coord1, const Coord& coord2) {
    return std::hypot(coord2.x - coord1.x, coord2.y - coord1.y);
}

 Coord newPosition(const Coord& coord, float dir, float dist) {
    return {
            coord.x + dist * std::cos(dir),
            coord.y + dist * std::sin(dir)
    };
}

Coord normalize(const Coord& coord) {
    float len = length(coord);
    if (std::abs(len) < EPSILON) {
        return { 0.0f, 0.0f };
    }

    return { coord.x / len, coord.y / len };
}

std::ostream& operator<<(std::ostream& os, const Coord& coord) {
    os << " [" << coord.x << ", " << coord.y << "]";
    return os;
}

DroneConfig readConfig() {
    std::ifstream configStream("config.json");
    if (!configStream) {
        std::cerr << "Error opening config.json\n";
        std::exit(1);
    }
    json configJson;
    configStream >> configJson;
    configStream.close();

    DroneConfig config;
    config.startPos.x    = configJson["drone"]["position"]["x"];
    config.startPos.y    = configJson["drone"]["position"]["y"];
    config.altitude      = configJson["drone"]["altitude"];
    config.initialDir    = configJson["drone"]["initialDirection"];
    config.attackSpeed   = configJson["drone"]["attackSpeed"];
    config.accelPath     = configJson["drone"]["accelerationPath"];
    config.angularSpeed  = configJson["drone"]["angularSpeed"];
    config.turnThreshold = configJson["drone"]["turnThreshold"];
    config.simTimeStep   = configJson["simulation"]["timeStep"];
    config.hitRadius     = configJson["simulation"]["hitRadius"];
    config.arrayTimeStep = configJson["targetArrayTimeStep"];

    std::string ammoNameStr = configJson["ammo"].get<std::string>();
    std::strncpy(config.ammoName, ammoNameStr.c_str(), MAX_NAME_LENGTH - 1);
    config.ammoName[MAX_NAME_LENGTH - 1] = '\0';

    return config;
}

AmmoParams readAmmoParams(const char* name) {
    std::ifstream ammoStream("ammo.json");
    if (!ammoStream) {
        std::cerr << "Error opening ammo.json\n";
        std::exit(1);
    }

    json ammoList;
    ammoStream >> ammoList;
    ammoStream.close();

    int ammoCount = ammoList.size();

    for (int i = 0; i < ammoCount; ++i) {
        std::string ammoName = ammoList[i]["name"].get<std::string>();
        if (std::strncmp(name, ammoName.c_str(), MAX_NAME_LENGTH) == 0) {
            AmmoParams ammoParams;
            std::strncpy(ammoParams.name, ammoName.c_str(), MAX_NAME_LENGTH - 1);
            ammoParams.name[MAX_NAME_LENGTH - 1] = '\0';
            ammoParams.mass = ammoList[i]["mass"];
            ammoParams.drag = ammoList[i]["drag"];
            ammoParams.lift = ammoList[i]["lift"];

            return ammoParams;
        }
    }
    std::cerr << "Error: Unknown ammo name " << name << "\n";
    std::exit(1);
}

float calcDirFromXAxis(const Coord& pos) {
    return std::atan2(pos.y, pos.x);
}

Coord calcDropPosition(const Coord& targetPos, const Coord& dronePos, float hDist) {
    Coord delta = targetPos - dronePos;
    float dist = length(delta);
    return targetPos - normalize(delta) * hDist;
}

float calcDroneFlightTime(float D, float attackSpeed, float accelerationPath) {
    float a = attackSpeed * attackSpeed / (2 * accelerationPath);

    if (D <= accelerationPath) {
        return std::sqrt(2 * D / a);
    } else {
        float accelerationTime = (2 * accelerationPath) / attackSpeed;
        return accelerationTime + (D - accelerationPath) / attackSpeed;
    }
}

float calcHDistance(const float m, const float d, const float l, const float zd, const float attackSpeed) {
    const float g = GRAVITY;

    float a = d * g * m - 2 * d * d * l * attackSpeed;
    float b = -3 * g * m * m + 3 * d * l * m * attackSpeed;
    float c = 6 * m * m * zd;

    float p = -(b * b) / (3 * a * a);
    float q = 2 * std::pow(b, 3) / (27 * std::pow(a, 3)) + c / a;

    float acos_arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

    if (acos_arg < -1 || acos_arg > 1) {
        std::cerr << "Error in calculations. Height is too big (" << acos_arg << ")\n";
        std::exit(1);
    }

    float phi = std::acos(acos_arg);
    float t = 2 * std::sqrt(-p / 3.0) * std::cos((phi + 4 * M_PI) / 3.0) - b / (3.0 * a);
    float h = attackSpeed * t -
        t * t * d * attackSpeed / (2 * m) + std::pow(t, 3) * (6 * d * g * l * m - 6 * d * d * (l * l - 1) * attackSpeed) / (36 * m * m) +
        std::pow(t, 4) * (-6 * d * d * g * l * (1 + l * l + std::pow(l, 4)) * m + 3 * std::pow(d, 3) * l * l * (1 + l * l) * attackSpeed + 6 * std::pow(d, 3) * std::pow(l, 4) * (1 + l * l) * attackSpeed) / (36 * std::pow((1 + l * l), 2) * std::pow(m, 3)) +
        std::pow(t, 5) * (3 * std::pow(d, 3) * g * std::pow(l, 3) * m - 3 * pow(d, 4) * l * l * (1 + l * l) * attackSpeed) / (36 * (1 + l * l) * std::pow(m, 4));

    return h;
}
