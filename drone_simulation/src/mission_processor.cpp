#include <cmath>

#include "coord.hpp"
#include "ammo_params.hpp"
#include "drone_config.hpp"
#include "mission_processor.hpp"


MissionProcessor::MissionProcessor(IBallisticSolver* solver, ITargetProvider* targetProvider, IConfigLoader* configLoader): 
solver{solver}, targetProvider{targetProvider}, configLoader{configLoader} {
}

MissionProcessor::~MissionProcessor() {
    delete solver;
    delete targetProvider;
    delete configLoader;
}

void MissionProcessor::reset() {
    currentTime = 0;
    currentStep = 0;

    DroneConfig config = configLoader->getConfig();
    simStep.speed = 0.0f;
    simStep.pos = config.startPos;
    simStep.direction = config.initialDir;
    simStep.state = DroneState::STOPPED;
    simStep.targetIdx = -1;
}

void MissionProcessor::init(const char* configfile, const char* ammofile, const char* targetsfile) {
    configLoader->load(configfile, ammofile);
    targetProvider->load(targetsfile);

    reset();
}

void MissionProcessor::changeSolver(IBallisticSolver* otherSolver) {
    solver = otherSolver;
}

bool MissionProcessor::hasNext() const {
    DroneConfig config = configLoader->getConfig();
    return currentStep < MAX_STEPS && distance(simStep.pos, simStep.dropPoint) > config.hitRadius;
}

int MissionProcessor::getCurrentStep() const {
    return currentStep;
}

SimStep MissionProcessor::step() {
    DroneConfig config = configLoader->getConfig();
    AmmoParams ammo = configLoader->getAmmoParams();

    int targetTimeSteps = targetProvider->getTargetTimeSteps();
    int targetCount = targetProvider->getTargetCount();

    int currentIdx = static_cast<int>(currentTime / config.arrayTimeStep) % targetTimeSteps;
    int nextIdx = (currentIdx + 1) % targetTimeSteps;
    float frac = (currentTime - currentIdx * config.arrayTimeStep) / config.arrayTimeStep;

    int prevTargetIdx = simStep.targetIdx;
    float minFireTime = INFINITY;

    for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
        Coord currTargetPos = targetProvider->getTarget(targetNum, currentIdx);
        Coord nextRawTargetPos = targetProvider->getTarget(targetNum, nextIdx);
        Coord interpolatedPos = currTargetPos + (nextRawTargetPos - currTargetPos) * frac;

        float targetFlightTime = solver->calcFlightTime(simStep.pos, interpolatedPos, config.attackSpeed, config.accelPath);
        Coord velocity = (nextRawTargetPos - currTargetPos) / config.arrayTimeStep;
        Coord predictedTarget = interpolatedPos + velocity * targetFlightTime;

        Coord dropPoint = solver->solve(simStep.pos, predictedTarget, config.altitude, ammo, config.attackSpeed);

        float fireTime = solver->calcFlightTime(simStep.pos, dropPoint, config.attackSpeed, config.accelPath);
        float nextDir = direction(dropPoint);

        if (targetNum != prevTargetIdx) {
            float deltaAngle = std::abs(nextDir - simStep.direction);
            if (deltaAngle > config.turnThreshold) {
                float decelerationTime = (2 * config.accelPath) / config.attackSpeed;
                fireTime += decelerationTime;
                fireTime += (deltaAngle / config.angularSpeed);
            }
        }

        if (fireTime < minFireTime) {
            simStep.dropPoint = dropPoint;
            simStep.predictedTarget = predictedTarget;
            simStep.targetIdx = targetNum;
            targetDirection = nextDir;

            minFireTime = fireTime;
        }
    }

    bool targetChanged = (simStep.targetIdx != prevTargetIdx && prevTargetIdx != -1);
    float deltaAngle = std::abs(targetDirection - simStep.direction);

    if (targetChanged && deltaAngle > config.turnThreshold) {
        turnStartTime = currentTime;
        turnDuration = deltaAngle / config.angularSpeed;

        if (simStep.state == DroneState::MOVING || simStep.state == DroneState::ACCELERATING) {
            simStep.state = DroneState::DECELERATING;
        }
    }

    float turnElapsed = currentTime - turnStartTime;
    float a = config.attackSpeed * config.attackSpeed / (2 * config.accelPath);

    switch (simStep.state) {
        case DroneState::STOPPED:
            simStep.state = DroneState::ACCELERATING;
            simStep.speed = std::min(config.attackSpeed, a * config.simTimeStep);
            simStep.direction = targetDirection;
            break;

        case DroneState::ACCELERATING:
            if (simStep.speed >= config.attackSpeed - EPSILON) {
                simStep.state = DroneState::MOVING;
                simStep.speed = config.attackSpeed;
            } else {
                simStep.speed = std::min(config.attackSpeed, simStep.speed + a * config.simTimeStep);
            }
            simStep.direction = targetDirection;
            break;

        case DroneState::MOVING:
            simStep.speed = config.attackSpeed;
            simStep.direction = targetDirection;
            break;

        case DroneState::DECELERATING:
            if (simStep.speed < EPSILON) {
                simStep.state = DroneState::TURNING;
                simStep.speed = 0.0f;
            } else {
                simStep.speed = std::max(0.0f, simStep.speed - a * config.simTimeStep);
            }
            break;

        case DroneState::TURNING:
            float decelerationTime = (2 * config.accelPath) / config.attackSpeed;
            if (turnElapsed >= decelerationTime + turnDuration) {
                simStep.state = DroneState::ACCELERATING;
                simStep.direction = targetDirection;
                simStep.speed = std::min(config.attackSpeed, a * config.simTimeStep);
            } else {
                float turnProgress = (turnElapsed - decelerationTime) / turnDuration;
                turnProgress = std::min(1.0f, std::max(0.0f, turnProgress));

                float angleDiff = targetDirection - simStep.direction;
                if (angleDiff > M_PI) angleDiff -= 2 * M_PI;
                if (angleDiff < -M_PI) angleDiff += 2 * M_PI;

                simStep.direction = simStep.direction + angleDiff * turnProgress;
                simStep.speed = 0.0f;
            }
            break;

        }

    currentStep += 1;
    currentTime += config.simTimeStep;

    simStep.aimPoint = solver->calcAimPoint(simStep.pos, simStep.direction, config.altitude, ammo, simStep.speed);
    simStep.pos.x += simStep.speed * std::cos(simStep.direction) * config.simTimeStep;
    simStep.pos.y += simStep.speed * std::sin(simStep.direction) * config.simTimeStep;

    return simStep;
}
