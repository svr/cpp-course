#include <cmath>
#include "state_accelerating.hpp"
#include "state_moving.hpp"
#include "state_decelerating.hpp"
#include "coord.hpp"

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    if (ctx.targetChanged) {
        float deltaAngle = std::abs(angleDifference(ctx.targetDirection, ctx.direction));
        if (deltaAngle > ctx.config.turnThreshold) {
            ctx.turnRemaining = deltaAngle / ctx.config.angularSpeed;
            ctx.turnStartDirection = ctx.direction;
            return std::make_unique<StateDecelerating>();
        }
        ctx.targetChanged = false;
    }

    if (ctx.speed >= ctx.config.attackSpeed - EPSILON) {
        ctx.speed = ctx.config.attackSpeed;
        return std::make_unique<StateMoving>();
    }

    ctx.speed = std::min(ctx.config.attackSpeed, ctx.speed + ctx.a * ctx.config.simTimeStep);
    return std::make_unique<StateAccelerating>();
}