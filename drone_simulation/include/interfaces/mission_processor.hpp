#pragma once
#include <string>
#include <memory>

#include "sim_step.hpp"
#include "ballistic_solver.hpp"
#include "target_provider.hpp"
#include "config_loader.hpp"

class MissionProcessor {
    private:
    std::unique_ptr<IBallisticSolver> solver;
    std::unique_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IConfigLoader> configLoader;

    float currentTime = 0;
    int currentStep = 0;
    float targetDirection = 0;
    float turnStartTime = 0;
    float turnDuration = 0;
    float turnStartDirection = 0;

    SimStep simStep;

    public:
    MissionProcessor(std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<ITargetProvider> targetProvider, std::unique_ptr<IConfigLoader> configLoader);
    void init(const std::string& configfile, const std::string& ammofile, const std::string& targetsfile);
    bool hasNext() const;
    SimStep step();
    int getCurrentStep() const;
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> otherSolver);
};