#pragma once
#include <memory>
#include "drone_context.hpp"
#include "drone_state.hpp"

struct DroneCommand {
	std::unique_ptr<IDroneState*> state;
	float angleSpeed;
};

