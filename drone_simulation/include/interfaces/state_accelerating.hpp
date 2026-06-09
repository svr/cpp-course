#pragma once

#include "drone_state.hpp"

class StateAccelerating : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override {
        return "Accelerating";
    }
    int id() const override {
        return 1;
    }
};
