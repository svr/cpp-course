#include <cmath>
#include "state_moving.hpp"
#include "state_decelerating.hpp"
#include "coord.hpp"

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    if (ctx.targetChanged) {
        float deltaAngle = std::abs(angleDifference(ctx.targetDirection, ctx.direction));
        if (deltaAngle > ctx.config.turnThreshold) {
            ctx.turnRemaining = deltaAngle / ctx.config.angularSpeed;
            ctx.turnStartDirection = ctx.direction;
            return std::make_unique<StateDecelerating>();
        }
        ctx.targetChanged = false;
    }

    ctx.speed = ctx.config.attackSpeed;
    return std::make_unique<StateMoving>();
}