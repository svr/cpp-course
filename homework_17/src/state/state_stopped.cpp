#include <cmath>

#include "coord.hpp"
#include "state_stopped.hpp"
#include "state_turning.hpp"
#include "state_accelerating.hpp"

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    float delta = angleDifference(ctx.targetDirection, ctx.direction);

    if (std::fabs(delta) > ctx.config.turnThreshold) {
        ctx.turnRemaining = std::fabs(delta) / ctx.config.angularSpeed;
        return std::make_unique<StateTurning>();
    }

    ctx.direction = ctx.targetDirection;
    return std::make_unique<StateAccelerating>();
}