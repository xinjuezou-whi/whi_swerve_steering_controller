/******************************************************************
wheel properties

Features:
- radius and position
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-07-11: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <cmath>
#include <algorithm>

class Wheel
{
public:
    Wheel() = default;
    Wheel(double Radius, const std::array<double, 2>& Position)
        : radius_(Radius), position_(std::move(Position)) {};
    ~Wheel() = default;

public:
    double offset_{ 0.0 };
    double radius_{ 0.1 };
    std::array<double, 2> position_{ 0.0, 0.0 };
};
