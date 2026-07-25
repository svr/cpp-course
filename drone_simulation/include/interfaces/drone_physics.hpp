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

class DronePhysics : public RunnableThread {
private:
    std::unique_ptr<IBallisticSolver> solver;
    std::shared_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IConfigLoader> configLoader;
    DroneContext ctx;
    std::unique_ptr<IDroneState> state;

    Coord pos;
    float totalSimulatedTime = 0.0f;
    float physicsTimeStep = 0.01f;
    float timeScale = 10.0f;

    DroneTelemetry currentTelemetry;
    mutable std::mutex physicsMutex;

    void updateTelemetry();

public:
    ~DronePhysics() = default;
    void init(const DroneConfig& config, float physicsStep, float scale,
              std::unique_ptr<IBallisticSolver> solver,
              std::shared_ptr<ITargetProvider> provider,
              std::unique_ptr<IConfigLoader> loader);

    void run() override;
    DroneContext step(float simTimeStep);
    int getCurrentStateId() const;
    DroneTelemetry getTelemetry() const;
};