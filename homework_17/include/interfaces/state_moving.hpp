#pragma once

#include "drone_state.hpp"

class StateMoving : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override {
        return "Moving";
    }
    int id() const override {
        return 4;
    }
};
