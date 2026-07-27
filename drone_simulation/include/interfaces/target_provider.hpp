#pragma once
#include "target.hpp"
#include "drone_config.hpp"
#include "runnable_thread.hpp"

class ITargetProvider : public RunnableThread {
public:
    virtual void init(const DroneConfig& cfg) { (void)cfg; }
    virtual void load() = 0;

    virtual int getTargetCount() const = 0;
    virtual Target getTarget(int targetNum) const = 0;
    virtual Coord getTarget(int num, int timeIndex) const = 0;
    virtual int getTargetTimeSteps() const = 0;
    virtual ~ITargetProvider() = default;
    void run() override {}
};