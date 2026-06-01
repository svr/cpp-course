#include <cmath>
#include <algorithm>

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

    float cycleTime = targetTimeSteps * config.arrayTimeStep;
    float localTime = std::fmod(currentTime, cycleTime);

    int currentIdx = static_cast<int>(localTime / config.arrayTimeStep);
    int nextIdx = (currentIdx + 1) % targetTimeSteps;

    float frac = (localTime - currentIdx * config.arrayTimeStep) / config.arrayTimeStep;

    int prevTargetIdx = simStep.targetIdx;
    float minFireTime = std::numeric_limits<float>::infinity();
    float bestDirection = simStep.direction;

    for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
        Coord currTargetPos = targetProvider->getTarget(targetNum, currentIdx);
        Coord nextTargetPos = targetProvider->getTarget(targetNum, nextIdx);
        Coord interpolatedPos = currTargetPos + (nextTargetPos - currTargetPos) * frac;

        float targetFlightTime = solver->calcFlightTime(
            simStep.pos,
            interpolatedPos,
            config.attackSpeed,
            config.accelPath);

        Coord velocity = (nextTargetPos - currTargetPos) / config.arrayTimeStep;
        Coord predictedTarget = interpolatedPos + velocity * targetFlightTime;

        Coord dropPoint = solver->solve(
            simStep.pos,
            predictedTarget,
            config.altitude,
            ammo,
            config.attackSpeed);

        float fireTime = solver->calcFlightTime(
            simStep.pos,
            dropPoint,
            config.attackSpeed,
            config.accelPath);

        float nextDir = direction(dropPoint - simStep.pos);

        if (targetNum != prevTargetIdx) {
            float deltaAngle = std::abs(angleDifference(nextDir, simStep.direction));
            if (deltaAngle > config.turnThreshold) {
                float decelerationTime = (2.0f * config.accelPath) / config.attackSpeed;
                fireTime += decelerationTime;
                fireTime += deltaAngle / config.angularSpeed;
            }
        }

        if (fireTime < minFireTime) {
            simStep.dropPoint = dropPoint;
            simStep.predictedTarget = predictedTarget;
            simStep.targetIdx = targetNum;

            bestDirection = nextDir;
            minFireTime = fireTime;
        }
    }

    targetDirection = bestDirection;

    bool targetChanged =simStep.targetIdx != prevTargetIdx && prevTargetIdx != -1;
    float deltaAngle = std::abs(angleDifference(targetDirection, simStep.direction));

    if (targetChanged &&
        deltaAngle > config.turnThreshold &&
        simStep.state != DroneState::DECELERATING &&
        simStep.state != DroneState::TURNING) {

        turnDuration = deltaAngle / config.angularSpeed;
        turnStartDirection = simStep.direction;

        if (simStep.state == DroneState::MOVING || simStep.state == DroneState::ACCELERATING) {
            simStep.state = DroneState::DECELERATING;
        }
    }

    float a = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);

    switch (simStep.state) {
        case DroneState::STOPPED:
            simStep.state = DroneState::ACCELERATING;
            simStep.direction = targetDirection;
            simStep.speed = std::min(config.attackSpeed, a * config.simTimeStep);
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
            simStep.speed = std::max(0.0f, simStep.speed - a * config.simTimeStep);
            if (simStep.speed <= EPSILON) {
                simStep.speed = 0.0f;
                turnStartTime = currentTime;

                if (turnDuration <= EPSILON) {
                    simStep.direction = targetDirection;
                    simStep.state = DroneState::ACCELERATING;
                } else {
                    simStep.state = DroneState::TURNING;
                }
            }
            break;

        case DroneState::TURNING: {
            float turnElapsed = currentTime - turnStartTime;

            if (turnElapsed >= turnDuration) {
                simStep.direction = targetDirection;
                simStep.state = DroneState::ACCELERATING;

                simStep.speed = std::min(config.attackSpeed, a * config.simTimeStep);
            } else {
                float turnProgress = turnDuration > EPSILON ? turnElapsed / turnDuration : 1.0f;
                turnProgress = std::clamp(turnProgress, 0.0f, 1.0f);

                float angleDiff = angleDifference(targetDirection, turnStartDirection);

                simStep.direction = turnStartDirection + angleDiff * turnProgress;
                simStep.speed = 0.0f;
            }
            break;
        }
    }

    currentStep += 1;
    currentTime += config.simTimeStep;

    simStep.aimPoint = solver->calcAimPoint(simStep.pos, simStep.direction, config.altitude, ammo, simStep.speed);
    simStep.pos.x += simStep.speed * std::cos(simStep.direction) * config.simTimeStep;
    simStep.pos.y += simStep.speed * std::sin(simStep.direction) * config.simTimeStep;

    return simStep;
}