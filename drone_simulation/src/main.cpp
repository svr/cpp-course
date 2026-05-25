#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include "drone_simulation.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define ENABLE_LOG    1
#define ENABLE_DEBUG  0


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