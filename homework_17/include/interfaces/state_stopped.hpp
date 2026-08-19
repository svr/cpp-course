#pragma once

#include "drone_state.hpp"

class StateStopped : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override {
        return "Stopped";
    }
    int id() const override {
        return 0;
    }
};
