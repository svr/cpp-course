#pragma once
#include <string>
#include <vector>
#include <mutex>

#include "common.hpp"
#include "coord.hpp"
#include "runnable_thread.hpp"
#include "target_provider.hpp"
#include "drone_config.hpp"

class ThreadSafeTargetProvider : public ITargetProvider, public RunnableThread {
private:
    int targetCount{0};
    int timeSteps{0};
    std::string targetsfile;
    std::vector<std::vector<Coord>> targets; // [targetIdx][nodeIdx]
    std::vector<Target> currentTargets;
    mutable std::mutex providerMutex;
    DroneConfig config;

    int currentIdx = 0;

    Target getCurrentTargetUnlocked(int targetNum) const {
        if (targetNum >= 0 && targetNum < (int)currentTargets.size()) {
            return currentTargets.at(targetNum);
        }
        return Target{};
    }
public:
    ThreadSafeTargetProvider(const std::string& targetsfile) : targetsfile{targetsfile} {};
    ~ThreadSafeTargetProvider() = default;

    void init(const DroneConfig& cfg) override {
        config = cfg;
    }

    void load() override;
    void run() override;

    bool isThreadReady() const override { return RunnableThread::isThreadReady(); }
    void start() override { RunnableThread::start(); }
    void stop() override { RunnableThread::stop(); }

    int getTargetTimeSteps() const override {
        std::lock_guard<std::mutex> lock(providerMutex);
        return timeSteps;
    }

    int getTargetCount() const override {
        std::lock_guard<std::mutex> lock(providerMutex);
        return currentTargets.size();
    }

    Coord getTarget(int num, int timeIndex) const override {
        std::lock_guard<std::mutex> lock(providerMutex);
        if (num >= 0 && num < (int)targets.size() && timeIndex >= 0 && timeIndex < (int)targets[num].size()) {
            return targets[num][timeIndex];
        }
        return getCurrentTargetUnlocked(num).pos;
    }

    Target getTarget(int targetNum) const override {
        std::lock_guard<std::mutex> lock(providerMutex);
        return getCurrentTargetUnlocked(targetNum);
    }
};