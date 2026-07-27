#pragma once
#include <mutex>
#include <memory>
#include "runnable_thread.hpp"
#include "coord.hpp"
#include "drone_config.hpp"
#include "drone_context.hpp"
#include "drone_telemetry.hpp"
#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "target_provider.hpp"
#include "drone_state.hpp"
#include "uart_data_provider.hpp"

class DroneUart : public RunnableThread {
private:
    std::unique_ptr<IBallisticSolver> solver;
    std::shared_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IConfigLoader> configLoader;
    std::shared_ptr<UartDataProvider> uartDataProvider;

    DroneContext ctx;
    std::unique_ptr<IDroneState> state;

    DroneTelemetry currentTelemetry;
    mutable std::mutex physicsMutex;

    void updateTelemetry();

public:
    ~DroneUart() = default;
    void init(std::unique_ptr<IBallisticSolver> solver,
              std::shared_ptr<ITargetProvider> provider,
              std::unique_ptr<IConfigLoader> loader,
              std::shared_ptr<UartDataProvider> uartProvider);

    DroneContext step(float simTimeStep);
    void run() override {}
    int getCurrentStateId() const;
    DroneTelemetry getTelemetry() const;
    void sendControl(const DroneContext& ctx, float simTimeStep) const;
};