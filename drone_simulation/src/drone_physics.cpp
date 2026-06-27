#include <cmath>
#include <limits>
#include <thread>
#include <chrono>

#include "drone_physics.hpp"
#include "state_stopped.hpp"

void DronePhysics::init(const DroneConfig& config, float physicsStep, float scale,
                        std::unique_ptr<IBallisticSolver> bSolver,
                        std::shared_ptr<ThreadSafeTargetProvider> tProvider,
                        std::unique_ptr<IConfigLoader> cLoader) {
    solver = std::move(bSolver);
    targetProvider = tProvider;
    configLoader = std::move(cLoader);

    solver->load();
    targetProvider->load();
    configLoader->load();

    pos = config.startPos;
    physicsTimeStep = physicsStep;
    timeScale = scale;

    state = std::make_unique<StateStopped>();

    ctx.config = config;
    ctx.speed = 0.0f;
    ctx.pos = config.startPos;
    ctx.direction = config.initialDir;
    ctx.targetIdx = -1;
    ctx.turnRemaining = 0.0f;
    ctx.targetChanged = false;

    currentTelemetry.pos = pos;
    currentTelemetry.speed = { 0.0f, 0.0f };
    currentTelemetry.timeSecSinceStart = 0.0f;

    isReady.store(true);
}

DroneContext DronePhysics::step(float simTimeStep) {
    DroneConfig config = configLoader->getConfig();
    AmmoParams ammo = configLoader->getAmmoParams();

    int targetCount = targetProvider->getTargetCount();
    std::vector<Target> targetSnapshots;
    targetSnapshots.reserve(targetCount);
    for (int i = 0; i < targetCount; ++i) {
        targetSnapshots.push_back(targetProvider->getTarget(i));
    }

    std::lock_guard<std::mutex> lock(physicsMutex);

    ctx.pos = pos;
    ctx.config.simTimeStep = simTimeStep;

    int prevTargetIdx = ctx.targetIdx;
    float minFireTime = std::numeric_limits<float>::infinity();
    float bestDirection = ctx.direction;

    for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
        const Target& targetSnapshot = targetSnapshots[targetNum];

        float targetFlightTime = solver->calcFlightTime(
            ctx.pos, targetSnapshot.pos, config.attackSpeed, config.accelPath);

        Coord predictedTarget = targetSnapshot.pos + targetSnapshot.velocity * targetFlightTime;

        Coord dropPoint = solver->solve(
            ctx.pos, predictedTarget, config.altitude, ammo, config.attackSpeed);

        float fireTime = solver->calcFlightTime(ctx.pos, dropPoint, config.attackSpeed, config.accelPath);
        float nextDir = direction(dropPoint - ctx.pos);

        if (targetNum != prevTargetIdx && prevTargetIdx != -1) {
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

    ctx.aimPoint = solver->calcAimPoint(ctx.pos, ctx.direction, config.altitude, ammo, ctx.speed);

    return ctx;
}

void DronePhysics::run() {
    while (!isStarted.load() && !shouldStop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto lastTime = std::chrono::steady_clock::now();
    float accumulator = 0.0f;

    while (!shouldStop.load()) {
        auto currentTime = std::chrono::steady_clock::now();
        float dtReal = std::chrono::duration<float>(currentTime - lastTime).count();
        lastTime = currentTime;

        accumulator += dtReal * timeScale;

        bool physicsUpdated = false;

        if (accumulator >= physicsTimeStep) {
            std::lock_guard<std::mutex> lock(physicsMutex);
            while (accumulator >= physicsTimeStep) {
                pos.x += ctx.speed * std::cos(ctx.direction) * physicsTimeStep;
                pos.y += ctx.speed * std::sin(ctx.direction) * physicsTimeStep;
                totalSimulatedTime += physicsTimeStep;

                accumulator -= physicsTimeStep;
                physicsUpdated = true;
            }

            if (physicsUpdated) {
                updateTelemetry();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

DroneTelemetry DronePhysics::getTelemetry() const {
    std::lock_guard<std::mutex> lock(physicsMutex);
    return currentTelemetry;
}

int DronePhysics::getCurrentStateId() const {
    std::lock_guard<std::mutex> lock(physicsMutex);
    return state ? state->id() : 0;
}

void DronePhysics::updateTelemetry() {
    currentTelemetry.pos = pos;
    currentTelemetry.speed = { ctx.speed * std::cos(ctx.direction), ctx.speed * std::sin(ctx.direction) };
    currentTelemetry.timeSecSinceStart = totalSimulatedTime;
}