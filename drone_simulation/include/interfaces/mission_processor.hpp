#pragma once
#include <string>

#include "ballistic_solver.hpp"
#include "target_provider.hpp"
#include "config_loader.hpp"

class MissionProcessor {
    private:
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IConfigLoader> configLoader;
    std::unique_ptr<IDroneState> state;


    float currentTime = 0;
    int currentStep = 0;
    float targetDirection = 0;
    float turnStartTime = 0;
    float turnDuration = 0;
    float turnStartDirection = 0;
    float a = 0;

    DroneContext ctx;

    public:
    MissionProcessor(IBallisticSolver* solver, ITargetProvider* targetProvider, IConfigLoader* configLoader);
    ~MissionProcessor();
    void init(const std::string& configfile, const std::string& ammofile, const std::string& targetsfile);
    bool hasNext() const;
    DroneContext step();
    int getCurrentStep() const;
    int getStateId() const;
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> otherSolver);
};