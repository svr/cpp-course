#pragma once
#include <memory>
#include <nlohmann/json.hpp>
#include "runnable_thread.hpp"
#include "drone_physics.hpp"
#include "drone_config.hpp"

class MissionProcessor : public RunnableThread {
private:
    std::shared_ptr<DronePhysics> dronePhysics;
    DroneConfig config;

    int currentStep = 0;
    const int MAX_STEPS = 10000;


    nlohmann::json simulationLog;

public:
    MissionProcessor(std::shared_ptr<DronePhysics> physics);
    virtual ~MissionProcessor() = default;
    void init(const DroneConfig&);
    void run() override;

    nlohmann::json getSimulationLog() const { return simulationLog; }
};