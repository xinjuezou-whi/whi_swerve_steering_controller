/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mark Naeem
 */
#pragma once
#include "utils.h"
#include "interval.h"

#include <cmath>
#include <algorithm>

class Wheel
{
public:
    Wheel() = default;
    Wheel(double Radius, const std::array<double, 2>& Position,
        bool Limitless = false, const std::array<double, 2>& RotationLimits = {-M_PI_2, M_PI_2});
    ~Wheel() = default;

    void set_rotation_limits(const std::array<double, 2>& RotationLimits = {-M_PI_2, M_PI_2}, bool Limitless = false);

    void set_current_angle(double Angle);
    double get_current_angle() const;

    bool set_command_angle(double Target);
    double get_command_angle() const;

    void set_command_velocity(double Velocity);
    double get_command_velocity() const;

    void set_limitless(bool Limitless);
    bool get_limitless() const;

    int get_omega_direction() const;

public:
    //for the controller use only. THis class wont use them by itself
    double offset_{ 0.0 };
    double radius_{ 0.1 };
    std::array<double, 2> position_{ 0.0, 0.0 };

protected:
    double theta_{ 0.0 };
    double command_omega_{ 0.0 };
    double steering_angle_{ 0.0 };
    int omega_direc_{ 1 };
    bool limitless_{ false };
    interval limits_;
};
