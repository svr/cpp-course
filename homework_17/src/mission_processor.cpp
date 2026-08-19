#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include "mission_processor.hpp"

namespace {
    constexpr double telemetryPeriodSec = 0.5; // 2 Hz

    constexpr double lat0 = 50.4501;
    constexpr double lon0 = 30.5234;

    float get_lat(float y) {
        return lat0 + (y / 111000.0);
    }

    float get_lon(float x) {
        return lon0 + (x / (111000.0 * std::cos(lat0 * M_PI / 180.0)));
    }

    // direction is heading in radians; convert to centidegrees normalized to [0, 35999]
    uint16_t get_hdg(double directionRad) {
        double deg = std::fmod(directionRad * 180.0 / M_PI, 360.0);
        if (deg < 0.0) {
            deg += 360.0;
        }
        return static_cast<uint16_t>(deg * 100.0);
    }
}

MissionProcessor::MissionProcessor(std::shared_ptr<DronePhysics> physics, std::shared_ptr<MavlinkCommunication> mavlink)
    : dronePhysics(std::move(physics)), mavlink(std::move(mavlink)) {}

void MissionProcessor::init(const DroneConfig& cfg) {
    config = cfg;
    currentStep = 0;
    simulationLog["steps"] = nlohmann::json::array();
    isReady = true;
}

void MissionProcessor::run() {
    while (!isStarted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    double nextTelemetryTimeSec = 0.0;

    while (currentStep < MAX_STEPS) {
        DroneContext snapshotCtx = dronePhysics->step(config.simTimeStep);

        DroneTelemetry telemetry = dronePhysics->getTelemetry();
        if (telemetry.timeSecSinceStart >= nextTelemetryTimeSec) {
            const uint32_t time_boot_ms = static_cast<uint32_t>(telemetry.timeSecSinceStart * 1000.0);

            mavlink->send_attitude(
                snapshotCtx.direction,
                0.0f,
                0.0f,
                time_boot_ms
            );

            mavlink->send_position(
                static_cast<int32_t>(get_lat(telemetry.pos.y) * 1e7),
                static_cast<int32_t>(get_lon(telemetry.pos.x) * 1e7),

                // meters -> millimeters
                static_cast<int32_t>(config.altitude * 1000.0f),

                // m/s -> cm/s
                static_cast<int16_t>(telemetry.speed.x * 100.0f),
                static_cast<int16_t>(telemetry.speed.y * 100.0f),

                get_hdg(snapshotCtx.direction),
                time_boot_ms
            );

            nextTelemetryTimeSec += telemetryPeriodSec;

            if (nextTelemetryTimeSec < telemetry.timeSecSinceStart) {
                nextTelemetryTimeSec = telemetry.timeSecSinceStart + telemetryPeriodSec;
            }
        }

        int currentStateId = dronePhysics->getCurrentStateId();

        float dx = snapshotCtx.pos.x - snapshotCtx.dropPoint.x;
        float dy = snapshotCtx.pos.y - snapshotCtx.dropPoint.y;
        float distToDrop = std::hypot(dx, dy);

        if (distToDrop <= config.hitRadius) {
            std::cout << "Simulation finished after " << currentStep << " steps\n";
            mavlink->send_drop(
                get_lat(snapshotCtx.dropPoint.y),
                get_lon(snapshotCtx.dropPoint.x),
                config.altitude // in meters
            );
            break;
        }

        nlohmann::json stepLog;
        stepLog["position"]         = {{"x", snapshotCtx.pos.x}, {"y", snapshotCtx.pos.y}};
        stepLog["direction"]        = snapshotCtx.direction;
        stepLog["state"]            = currentStateId;
        stepLog["targetIndex"]      = snapshotCtx.targetIdx;
        stepLog["dropPoint"]        = {{"x", snapshotCtx.dropPoint.x}, {"y", snapshotCtx.dropPoint.y}};
        stepLog["aimPoint"]         = {{"x", snapshotCtx.aimPoint.x},  {"y", snapshotCtx.aimPoint.y}};
        stepLog["predictedTarget"]  = {{"x", snapshotCtx.predictedTarget.x}, {"y", snapshotCtx.predictedTarget.y}};
        stepLog["timeSecSinceStart"] = telemetry.timeSecSinceStart;

        simulationLog["steps"].push_back(stepLog);
        currentStep += 1;

        float sleepDurationSec = config.simTimeStep / config.timeScale;
        std::this_thread::sleep_for(std::chrono::duration<float>(sleepDurationSec));
    }
    simulationLog["totalSteps"] = currentStep;
}