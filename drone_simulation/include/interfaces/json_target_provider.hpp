#pragma once
#include <string>
#include <vector>

#include "common.hpp"
#include "coord.hpp"
#include "target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    int targetCount{0};
    int timeSteps{0};
    std::vector<std::vector<Coord>> targets;

    public:
    ~JsonTargetProvider() = default;
    void load(const std::string& file) override;
    int getTargetCount() const override;
    int getTargetTimeSteps() const override;
    Coord getTarget(int num, int timeIndex) const override;
};
