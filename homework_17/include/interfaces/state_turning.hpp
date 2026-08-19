#pragma once

#include "drone_state.hpp"

class StateTurning : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override {
        return "Turning";
    }
    int id() const override {
        return 3;
    }
};
