#pragma once
#include "coord.hpp"

struct DroneTelemetry {
	Coord pos;
	Coord speed;
	float timeSecSinceStart;
};
