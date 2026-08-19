#pragma once
#include <string>
#include <vector>

#include "common.hpp"
#include "coord.hpp"
#include "target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    int targetCount{0};
    int timeSteps{0};
    std::string targetsfile;
    std::vector<std::vector<Coord>> targets;

    public:
    JsonTargetProvider(const std::string& targetsfile): targetsfile{targetsfile} {};
    ~JsonTargetProvider() = default;
    void load() override;
    int getTargetCount() const override;
    Target getTarget(int targetNum) const override;
    int getTargetTimeSteps() const override;
    Coord getTarget(int num, int timeIndex) const override;
};
