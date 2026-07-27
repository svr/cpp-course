#pragma once
#include <string>
#include <vector>

#include "common.hpp"
#include "coord.hpp"
#include "target_provider.hpp"
#include "uart_data_provider.hpp"

class UartTargetProvider : public ITargetProvider {
    std::shared_ptr<UartDataProvider> uartDataProvider;
    std::vector<Target> targets;
    mutable std::mutex providerMutex;
    DroneConfig config;
    float timeStep = 0.0f;

    public:
    UartTargetProvider(std::shared_ptr<UartDataProvider> uartDataProvider, int count, float timeStep)
        : uartDataProvider{std::move(uartDataProvider)}, targets(count), timeStep(timeStep) {};
    ~UartTargetProvider() = default;
    void load() override;
    void run() override;
    int getTargetCount() const override { return static_cast<int>(targets.size()); }
    Target getTarget(int targetNum) const override;
    void init(const DroneConfig& cfg) override {
        config = cfg;
    }
    int getTargetTimeSteps() const override { return static_cast<int>(timeStep); };
    Coord getTarget(int num, int timeIndex) const override;
};
