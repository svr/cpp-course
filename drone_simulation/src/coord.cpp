#include <cmath>

#include "common.hpp"
#include "coord.hpp"


Coord& Coord::operator+=(const Coord& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Coord& Coord::operator-=(const Coord& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Coord& Coord::operator*=(float s) {
    x *= s;
    y *= s;
    return *this;
}

Coord& Coord::operator/=(float s) {
    if (std::abs(s) < EPSILON) {
        return *this;
    }
    x /= s;
    y /= s;
    return *this;
}

Coord Coord::operator+(const Coord& other) const {
    Coord result(*this);
    result += other;
    return result;
}

Coord Coord::operator-(const Coord& other) const {
    Coord result(*this);
    result -= other;
    return result;
}

Coord Coord::operator*(float s) const {
    Coord result(*this);
    result *= s;
    return result;
}

Coord Coord::operator/(float s) const {
    Coord result(*this);
    result /= s;
    return result;
}

bool Coord::operator==(const Coord& other) const {
    return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
}

float length(const Coord& coord) {
    return std::hypot(coord.x, coord.y);
}

float distance(const Coord& coord1, const Coord& coord2) {
    return std::hypot(coord2.x - coord1.x, coord2.y - coord1.y);
}

float direction(const Coord& coord) {
    return std::atan2(coord.y, coord.x);
}

Coord normalize(const Coord& coord) {
    float len = length(coord);
    if (std::abs(len) < EPSILON) {
        return { 0.0f, 0.0f };
    }

    return { coord.x / len, coord.y / len };
}

// std::ostream& operator<<(std::ostream& os, const Coord& coord) {
//     os << " [" << coord.x << ", " << coord.y << "]";
//     return os;
// }
