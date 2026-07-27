#pragma once
#include <memory>
#include <nlohmann/json.hpp>
#include "runnable_thread.hpp"
#include "gpio.hpp"
#include "drone_uart.hpp"
#include "drone_config.hpp"

class MissionProcessor : public RunnableThread {
private:
    std::shared_ptr<DroneUart> dronePhysics;
    DroneConfig config;
    std::shared_ptr<GPIO> gpio;

    int currentStep = 0;
    const int MAX_STEPS = 10000;


    nlohmann::json simulationLog;

public:
    MissionProcessor(std::shared_ptr<DroneUart> physics, std::shared_ptr<GPIO> gpio)
        : dronePhysics(std::move(physics)), gpio(std::move(gpio)) {};
    virtual ~MissionProcessor() = default;
    void init(const DroneConfig&);
    void run() override;

    nlohmann::json getSimulationLog() const { return simulationLog; }
};