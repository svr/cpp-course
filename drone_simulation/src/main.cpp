#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;


#define ENABLE_LOG    1
#define ENABLE_DEBUG  0

constexpr float EPSILON = 1e-6f;
constexpr float GRAVITY = 9.81f;

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif


struct Coord {
    float x;
    float y;

    Coord& operator+=(const Coord& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Coord& operator-=(const Coord& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Coord& operator*=(float s) {
        x *= s;
        y *= s;
        return *this;
    }

    Coord& operator/=(float s) {
        if (std::abs(s) < EPSILON) {
            return *this;
        }
        x /= s;
        y /= s;
        return *this;
    }

    Coord operator+(const Coord& other) const {
        Coord result(*this);
        result += other;
        return result;
    }

    Coord operator-(const Coord& other) const {
        Coord result(*this);
        result -= other;
        return result;
    }

    Coord operator*(float s) const {
        Coord result(*this);
        result *= s;
        return result;
    }

    Coord operator/(float s) const {
        Coord result(*this);
        result /= s;
        return result;
    }


    bool operator==(const Coord& other) const {
        return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
    }
};

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


constexpr int MAX_NAME_LENGTH = 32;
struct AmmoParams {
    char name[MAX_NAME_LENGTH];
    float mass;
    float drag;
    float lift;
};

enum DroneState {
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char  ammoName[MAX_NAME_LENGTH];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int   state;
    int   targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};


constexpr int MAX_STEPS = 10000;

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

int main() {
    DroneConfig config = readConfig();
    AmmoParams ammoParams = readAmmoParams(config.ammoName);


    const float H = calcHDistance(ammoParams.mass, ammoParams.drag, ammoParams.lift, config.altitude, config.attackSpeed);
    const float a = (config.attackSpeed * config.attackSpeed) / (2.0f * config.accelPath);

    #ifdef ENABLE_DEBUG
        std::cout << config.ammoName << "  mass = " << ammoParams.mass << "  drag = " << ammoParams.drag << "  lift = " << ammoParams.lift << "  H = " << H << "\n";
        std::cout << "attackSpeed = " << config.attackSpeed << "  accelerationPath = " << config.accelPath << "  a = " << a << "\n";
    #endif


    std::ifstream targetsfs("targets.json");
    if (!targetsfs) {
        std::cerr << "Error opening targets.json\n";
        return 1;
    }

    json targetsJson;
    targetsfs >> targetsJson;
    targetsfs.close();

    int targetCount = targetsJson["targetCount"];
    int timeSteps = targetsJson["timeSteps"];

    DEBUG(targetCount);
    DEBUG(timeSteps);

    Coord** targets = new Coord* [targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
            targets[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
        }
    }

    float currentTime = 0;
    int currentStep = 0;
    SimStep simStep;
    simStep.pos = config.startPos;
    simStep.direction = config.initialDir;
    simStep.state = STOPPED;
    simStep.targetIdx = -1;

    float currentSpeed = 0;
    float targetDirection = config.initialDir;
    float turnStartTime = 0;
    float turnDuration = 0;

    json simulationResult;
    simulationResult["steps"] = json::array();

    while (currentStep < MAX_STEPS) {
        int prevTargetNum = simStep.targetIdx;
        float prevDir = simStep.direction;

        float minFireTime = INFINITY;
        float minFireDistance = INFINITY;

        for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
            int idx = static_cast<int>(currentTime / config.arrayTimeStep) % timeSteps;
            int nextIdx = (idx + 1) % timeSteps;

            float frac = (currentTime - idx * config.arrayTimeStep) / config.arrayTimeStep;

            Coord currTargetPos = targets[targetNum][idx];
            Coord nextTargetPos = currTargetPos + (targets[targetNum][nextIdx] - currTargetPos) * frac;

            float D = distance(simStep.pos, currTargetPos);
            float totalTime = calcDroneFlightTime(D, config.attackSpeed, config.accelPath);

            Coord predictedTarget = currTargetPos + ((nextTargetPos - currTargetPos) / config.simTimeStep) * totalTime;
            Coord dropPoint = calcDropPosition(predictedTarget, simStep.pos, H);


            float fireDistance = distance(dropPoint, simStep.pos);
            float fireTime = calcDroneFlightTime(fireDistance, config.attackSpeed, config.accelPath);
            float nextDir = calcDirFromXAxis(dropPoint);

            if (targetNum != prevTargetNum) {
                float deltaAngle = std::abs(nextDir - prevDir);
                if (deltaAngle > config.turnThreshold) {
                    float decelerationTime = (2 * config.accelPath) / config.attackSpeed;
                    fireTime += decelerationTime;
                    fireTime += (deltaAngle / config.angularSpeed);
                }
            }

            if (fireTime < minFireTime) {
                simStep.dropPoint = dropPoint;
                simStep.predictedTarget = predictedTarget;
                simStep.aimPoint = newPosition(simStep.pos, nextDir, H);

                simStep.targetIdx = targetNum;
                targetDirection = nextDir;

                minFireTime = fireTime;
                minFireDistance = fireDistance;
            }
        }

        bool targetChanged = (simStep.targetIdx != prevTargetNum && prevTargetNum != -1);
        float deltaAngle = std::abs(targetDirection - simStep.direction);

        if (targetChanged && deltaAngle > config.turnThreshold) {
            turnStartTime = currentTime;
            turnDuration = deltaAngle / config.angularSpeed;

            if (simStep.state == MOVING || simStep.state == ACCELERATING) {
                simStep.state = DECELERATING;
            }
        }

        float turnElapsed = currentTime - turnStartTime;
        float decelerationTime = (2 * config.accelPath) / config.attackSpeed;

        switch (simStep.state) {
            case STOPPED:
                simStep.state = ACCELERATING;
                currentSpeed = std::min(config.attackSpeed, a * config.simTimeStep);
                simStep.direction = targetDirection;
                break;

            case ACCELERATING:
                if (currentSpeed >= config.attackSpeed - EPSILON) {
                    simStep.state = MOVING;
                    currentSpeed = config.attackSpeed;
                } else {
                    currentSpeed = std::min(config.attackSpeed, currentSpeed + a * config.simTimeStep);
                }
                simStep.direction = targetDirection;
                break;

            case MOVING:
                currentSpeed = config.attackSpeed;
                simStep.direction = targetDirection;
                break;

            case DECELERATING:
                if (currentSpeed < EPSILON) {
                    simStep.state = TURNING;
                    currentSpeed = 0.0f;
                } else {
                    currentSpeed = std::max(0.0f, currentSpeed - a * config.simTimeStep);
                }
                break;

            case TURNING:
                if (turnElapsed >= decelerationTime + turnDuration) {
                    simStep.state = ACCELERATING;
                    simStep.direction = targetDirection;
                    currentSpeed = std::min(config.attackSpeed, a * config.simTimeStep);
                } else {
                    float turnProgress = (turnElapsed - decelerationTime) / turnDuration;
                    turnProgress = std::min(1.0f, std::max(0.0f, turnProgress));

                    float angleDiff = targetDirection - prevDir;
                    if (angleDiff > M_PI) angleDiff -= 2 * M_PI;
                    if (angleDiff < -M_PI) angleDiff += 2 * M_PI;

                    simStep.direction = prevDir + angleDiff * turnProgress;
                    currentSpeed = 0.0f;
                }
                break;
        }

        currentStep += 1;
        currentTime += config.simTimeStep;

        simStep.pos.x += currentSpeed * std::cos(simStep.direction) * config.simTimeStep;
        simStep.pos.y += currentSpeed * std::sin(simStep.direction) * config.simTimeStep;

        json step;
        step["position"]        = {{"x", simStep.pos.x}, {"y", simStep.pos.y}};
        step["direction"]       = simStep.direction;
        step["state"]           = simStep.state;
        step["targetIndex"]     = simStep.targetIdx;
        step["dropPoint"]       = {{"x", simStep.dropPoint.x},       {"y", simStep.dropPoint.y}};
        step["aimPoint"]        = {{"x", simStep.aimPoint.x},        {"y", simStep.aimPoint.y}};
        step["predictedTarget"] = {{"x", simStep.predictedTarget.x}, {"y", simStep.predictedTarget.y}};

        simulationResult["steps"].push_back(step);


        if (minFireDistance <= config.hitRadius) {
            std::cout << "minFireDistance: " << minFireDistance << " hitRadius: " << config.hitRadius << "\n";
            std::cout << "Simulation finished after " << currentStep << " steps\n";
            break;
        }
    }

    for (int i = 0; i < targetCount; i++) {
        delete[] targets[i];
    }
    delete[] targets;

    std::ofstream simfstream("simulation.json");
    if (!simfstream) {
        std::cerr << "Error accessing simulation.json\n";
        std::exit(1);
    }

    simulationResult["totalSteps"] = currentStep;
    simfstream << simulationResult;
    simfstream.close();
}