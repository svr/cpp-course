#pragma once

struct Coord {
    float x;
    float y;

    Coord& operator+=(const Coord& other);
    Coord& operator-=(const Coord& other);
    Coord& operator*=(float s);
    Coord& operator/=(float s);
    Coord  operator+(const Coord& other) const;
    Coord  operator-(const Coord& other) const;
    Coord  operator*(float s) const;
    Coord  operator/(float s) const;
    bool   operator==(const Coord& other) const;
};

float length(const Coord& coord);
float distance(const Coord& coord1, const Coord& coord2);
float direction(const Coord& coord);
float angleDifference(float a, float b);
Coord newPosition(const Coord& coord, float dir, float dist);
Coord normalize(const Coord& coord);
