#include <cmath>
#include <string>
#include <algorithm>

#include "coord.hpp"
#include "ammo_params.hpp"
#include "drone_config.hpp"
#include "drone_context.hpp"
#include "state_stopped.hpp"
#include "mission_processor.hpp"


MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<ITargetProvider> targetProvider, std::unique_ptr<IConfigLoader> configLoader):
solver{std::move(solver)}, targetProvider{std::move(targetProvider)}, configLoader{std::move(configLoader)} {
}

void MissionProcessor::reset() {
    currentTime = 0;
    currentStep = 0;

    DroneConfig config = configLoader->getConfig();
    state = std::make_unique<StateStopped>();
    ctx.config = config;
    ctx.speed = 0.0f;
    ctx.a     = 0.0f;
    ctx.pos = config.startPos;
    ctx.direction = config.initialDir;
    ctx.targetIdx = -1;
}

void MissionProcessor::init(const std::string& configfile, const std::string& ammofile, const std::string& targetsfile) {
    configLoader->load(configfile, ammofile);
    targetProvider->load(targetsfile);

    reset();
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> otherSolver) {
    solver = std::move(otherSolver);
}

bool MissionProcessor::hasNext() const {
    DroneConfig config = configLoader->getConfig();
    return currentStep < MAX_STEPS && distance(ctx.pos, ctx.dropPoint) > config.hitRadius;
}

int MissionProcessor::getCurrentStep() const {
    return currentStep;
}

int MissionProcessor::getStateId() const {
    return state->id();
}

DroneContext MissionProcessor::step() {
    DroneConfig config = configLoader->getConfig();
    AmmoParams ammo = configLoader->getAmmoParams();

    int targetTimeSteps = targetProvider->getTargetTimeSteps();
    int targetCount = targetProvider->getTargetCount();

    float cycleTime = targetTimeSteps * config.arrayTimeStep;
    float localTime = std::fmod(currentTime, cycleTime);

    int currentIdx = static_cast<int>(localTime / config.arrayTimeStep);
    int nextIdx = (currentIdx + 1) % targetTimeSteps;

    float frac = (localTime - currentIdx * config.arrayTimeStep) / config.arrayTimeStep;

    int prevTargetIdx = ctx.targetIdx;
    float minFireTime = std::numeric_limits<float>::infinity();
    float bestDirection = ctx.direction;

    for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
        Coord currTargetPos = targetProvider->getTarget(targetNum, currentIdx);
        Coord nextTargetPos = targetProvider->getTarget(targetNum, nextIdx);
        Coord interpolatedPos = currTargetPos + (nextTargetPos - currTargetPos) * frac;

        float targetFlightTime = solver->calcFlightTime(
            ctx.pos,
            interpolatedPos,
            config.attackSpeed,
            config.accelPath);

        Coord velocity = (nextTargetPos - currTargetPos) / config.arrayTimeStep;
        Coord predictedTarget = interpolatedPos + velocity * targetFlightTime;

        Coord dropPoint = solver->solve(
            ctx.pos,
            predictedTarget,
            config.altitude,
            ammo,
            config.attackSpeed);

        float fireTime = solver->calcFlightTime(
            ctx.pos,
            dropPoint,
            config.attackSpeed,
            config.accelPath);

        float nextDir = direction(dropPoint - ctx.pos);

        if (targetNum != prevTargetIdx) {
            float deltaAngle = std::abs(angleDifference(nextDir, ctx.direction));
            if (deltaAngle > config.turnThreshold) {
                float decelerationTime = (2.0f * config.accelPath) / config.attackSpeed;
                fireTime += decelerationTime;
                fireTime += deltaAngle / config.angularSpeed;
            }
        }

        if (fireTime < minFireTime) {
            ctx.dropPoint = dropPoint;
            ctx.predictedTarget = predictedTarget;
            ctx.targetIdx = targetNum;

            bestDirection = nextDir;
            minFireTime = fireTime;
        }
    }

    ctx.targetDirection = bestDirection;

    ctx.targetChanged = (ctx.targetIdx != prevTargetIdx && prevTargetIdx != -1);

    ctx.a = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);

    state = std::move(state->execute(ctx));

    currentStep += 1;
    currentTime += config.simTimeStep;

    ctx.aimPoint = solver->calcAimPoint(ctx.pos, ctx.direction, config.altitude, ammo, ctx.speed);
    ctx.pos.x += ctx.speed * std::cos(ctx.direction) * config.simTimeStep;
    ctx.pos.y += ctx.speed * std::sin(ctx.direction) * config.simTimeStep;

    return ctx;
}
