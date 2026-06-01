#pragma once
#include <string>

#include "coord.hpp"

class ITargetProvider {
public:
    virtual void load(const std::string& file) = 0;
    virtual int getTargetCount() const = 0;
    virtual int getTargetTimeSteps() const = 0;
    virtual Coord getTarget(int num, int timeIndex) const = 0;

    virtual ~ITargetProvider() = default;
};