#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>

#include "drone_uart.hpp"
#include "state_stopped.hpp"

void DroneUart::init(std::unique_ptr<IBallisticSolver> bSolver,
                        std::shared_ptr<ITargetProvider> tProvider,
                        std::unique_ptr<IConfigLoader> cLoader,
                        std::shared_ptr<UartDataProvider> uartProvider) {
    solver = std::move(bSolver);
    targetProvider = tProvider;
    configLoader = std::move(cLoader);
    uartDataProvider = uartProvider;
    ctx.config = configLoader->getConfig();
    ctx.targetIdx = -1;
    ctx.turnRemaining = 0.0f;
    ctx.targetChanged = false;

    updateTelemetry();

    state = std::make_unique<StateStopped>();
    isReady.store(true);
}

DroneContext DroneUart::step(float simTimeStep) {
    updateTelemetry();

    DroneConfig config = configLoader->getConfig();
    AmmoParams ammo = configLoader->getAmmoParams();

    int targetCount = targetProvider->getTargetCount();
    std::vector<Target> targetSnapshots;
    targetSnapshots.reserve(targetCount);
    for (int i = 0; i < targetCount; ++i) {
        targetSnapshots.push_back(targetProvider->getTarget(i));
    }

    std::lock_guard<std::mutex> lock(physicsMutex);

    int prevTargetIdx = ctx.targetIdx;
    float minFireTime = std::numeric_limits<float>::infinity();
    float bestDirection = ctx.direction;

    for (int targetNum = 0; targetNum < targetCount; ++targetNum) {
        const Target& targetSnapshot = targetSnapshots[targetNum];

        float targetFlightTime = solver->calcFlightTime(
            ctx.pos, targetSnapshot.pos, config.attackSpeed, config.accelPath);

        Coord predictedTarget = targetSnapshot.pos + targetSnapshot.velocity * targetFlightTime;

        Coord dropPoint = solver->solve(
            ctx.pos, predictedTarget, ctx.config.altitude, ammo, config.attackSpeed);

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

    ctx.aimPoint = solver->calcAimPoint(ctx.pos, ctx.direction, ctx.config.altitude, ammo, ctx.speed);

    return ctx;
}

DroneTelemetry DroneUart::getTelemetry() const {
    std::lock_guard<std::mutex> lock(physicsMutex);
    return currentTelemetry;
}

int DroneUart::getCurrentStateId() const {
    std::lock_guard<std::mutex> lock(physicsMutex);
    return state ? state->id() : 0;
}

void DroneUart::updateTelemetry() {
    while (true) {
        std::optional<dlink::Telemetry> packet = uartDataProvider->readTelemetryPacket();
        if (!packet.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        std::lock_guard<std::mutex> lock(physicsMutex);
        const dlink::Telemetry telemetry = *packet;
        ctx.pos.x = telemetry.x;
        ctx.pos.y = telemetry.y;
        ctx.direction = telemetry.dir;
        ctx.speed = std::hypot(telemetry.vx, telemetry.vy);
        ctx.config.altitude = telemetry.z;

        currentTelemetry.pos = ctx.pos;
        currentTelemetry.speed = { telemetry.vx, telemetry.vy };
        currentTelemetry.timeSecSinceStart = telemetry.t_ms / 1000.0f;

        break;
    }
}

void DroneUart::sendControl(const DroneContext& ctx, float simTimeStep) const {
    constexpr float kControlEpsilon = 1e-3f;

    const float desiredDirection = direction(ctx.dropPoint - ctx.pos);
    const float headingError = angleDifference(desiredDirection, ctx.direction);
    const float headingErrorAbs = std::abs(headingError);

    const float turnThreshold = std::max(ctx.config.turnThreshold, kControlEpsilon);
    float turnRate = std::clamp(headingError / turnThreshold, -1.0f, 1.0f);

    const bool shouldBrakeForTurn = headingErrorAbs > ctx.config.turnThreshold;
    const float desiredSpeed = shouldBrakeForTurn ? 0.0f : ctx.config.attackSpeed;
    const float speedError = desiredSpeed - ctx.speed;

    const float dt = std::max(simTimeStep, kControlEpsilon);
    const float maxAccelPerTick = std::max(ctx.a * dt, kControlEpsilon);
    float accel = std::clamp(speedError / maxAccelPerTick, -1.0f, 1.0f);

    if (std::abs(turnRate) < kControlEpsilon) {
        turnRate = 0.0f;
    }
    if (std::abs(accel) < kControlEpsilon) {
        accel = 0.0f;
    }

    uartDataProvider->sendControlPacket(accel, turnRate);
}