#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include "mission_processor.hpp"


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

    while (currentStep < MAX_STEPS) {
        DroneContext snapshotCtx = dronePhysics->step(config.simTimeStep / config.timeScale);

        DroneTelemetry telemetry = dronePhysics->getTelemetry();
        int currentStateId = dronePhysics->getCurrentStateId();

        float dx = snapshotCtx.aimPoint.x - snapshotCtx.predictedTarget.x;
        float dy = snapshotCtx.aimPoint.y - snapshotCtx.predictedTarget.y;
        float impactError = std::hypot(dx, dy);

        if (impactError <= config.hitRadius) {
            std::cout << "Distance to drop point: " << impactError << ", Hit radius: " << config.hitRadius << ", Target index: " << snapshotCtx.targetIdx << std::endl;
            std::cout << "Simulation finished after " << currentStep << " steps\n";
            gpio->drop();
            simulationLog["totalSteps"] = currentStep;
            return;
        }

        dronePhysics->sendControl(snapshotCtx, config.simTimeStep);

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