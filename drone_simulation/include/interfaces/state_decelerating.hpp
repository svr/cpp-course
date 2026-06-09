#pragma once

#include "drone_state.hpp"

class StateDecelerating : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override {
        return "Decelerating";
    }
    int id() const override {
        return 2;
    }
};
