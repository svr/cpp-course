#include <cmath>


#include "state_turning.hpp"
#include "state_accelerating.hpp"

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    ctx.turnRemaining -= ctx.config.simTimeStep;

    if (ctx.turnRemaining > 0) {
        return std::make_unique<StateTurning>();
    }

    ctx.direction = ctx.targetDirection;
    ctx.turnRemaining = 0;
    return std::make_unique<StateAccelerating>();
}