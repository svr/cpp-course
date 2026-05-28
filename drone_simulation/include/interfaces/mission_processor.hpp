#pragma once
#include "sim_step.hpp"
#include "ballistic_solver.hpp"
#include "target_provider.hpp"
#include "config_loader.hpp"

class MissionProcessor {
    private:
    IBallisticSolver* solver;
    ITargetProvider*  targetProvider;
    IConfigLoader*    configLoader;

    float currentTime = 0;
    int currentStep = 0;
    float targetDirection = 0;
    float turnStartTime = 0;
    float turnDuration = 0;


    SimStep simStep;

    public:
    MissionProcessor(IBallisticSolver* solver, ITargetProvider* targetProvider, IConfigLoader* configLoader);
    ~MissionProcessor();
    void init(const char* configfile, const char* ammofile, const char* targetsfile);
    bool hasNext() const;
    SimStep step();
    int getCurrentStep() const;
    void reset();
    void changeSolver(IBallisticSolver* otherSolver);
};