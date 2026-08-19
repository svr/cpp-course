#include "state_decelerating.hpp"
#include "state_accelerating.hpp"
#include "state_turning.hpp"

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
    ctx.speed = std::max(0.0f, ctx.speed - ctx.a * ctx.config.simTimeStep);

    if (ctx.speed <= EPSILON) {
        ctx.speed = 0.0f;
        ctx.targetChanged = false;

        if (ctx.turnRemaining <= EPSILON) {
            ctx.direction = ctx.targetDirection;
            return std::make_unique<StateAccelerating>();
        } else {
            return std::make_unique<StateTurning>();
        }
    }

    return std::make_unique<StateDecelerating>();
}