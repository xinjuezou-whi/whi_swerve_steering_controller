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

#include "whi_swerve_steering_controller/wheel.h"

Wheel::Wheel(double Radius, const std::array<double, 2>& Position,
    bool Limitless, const std::array<double, 2>& RotationLimits)
    : radius_{Radius}, limitless_{Limitless}, limits_{interval(RotationLimits, "close").complement()} 
{
    position_ = std::move(Position);
}

void Wheel::set_rotation_limits(const std::array<double, 2>& RotationLimits, bool Limitless)
{
    limitless_ = Limitless;
    if (limitless_)
    {
        return;
    }
    limits_ = interval(RotationLimits, "close").complement();
}

bool Wheel::set_command_angle(double Target)
{
    bool triple_point = false;
    if (utils::isclose(Target, 0) || utils::isclose(Target, M_PI) || utils::isclose(Target, -M_PI))
    {
        triple_point = true;        
    }
    auto supplementary = utils::theta_map(Target + M_PI);        
    double supplementary2 = 0;
    if (triple_point)
    {
        supplementary2 = utils::isclose(supplementary, M_PI) ? -M_PI : M_PI;
    }

    auto interval1  = interval(std::array<double, 2>({theta_, Target}), "close");
    auto interval2  = interval(std::array<double, 2>({theta_, supplementary}), "close");
    auto interval3  = interval1.complement();
    auto interval4  = interval2.complement();
    
    interval interval5({-M_PI_2, M_PI_2}, "open"); // just initialized with any value, gonna be overwritten when used 
    interval interval6({-M_PI_2, M_PI_2}, "open"); // just initialized with any value, gonna be overwritten when used 
    if (triple_point)
    {
        interval5  = interval(std::array<double, 2>({theta_, supplementary2}), "close");
        interval6  = interval5.complement();
    }
    std::vector<double> lengths = {interval1.len(), interval3.len(), interval2.len(), interval4.len()};

    int min_arg;
    if (limitless_)
    {
        if (triple_point)
        {
            lengths.push_back(interval5.len());
            lengths.push_back(interval6.len());
        }

        min_arg = std::distance(lengths.begin(), std::min_element(lengths.begin(), lengths.end()));        
        if (min_arg < 2)
        {
            omega_direc_ = 1;
            this->steering_angle_ = Target;
            return true;
        }
        if (min_arg < 4)
        {
            omega_direc_ = -1;
            this->steering_angle_ = supplementary;
            return true;
        }
        else
        {
            omega_direc_ = -1;
            this->steering_angle_ = supplementary2;
            return true;
        }
    }
    else
    {
        std::vector<bool> intersects = {
            interval1.is_intersecting(limits_),
            interval3.is_intersecting(limits_),
            interval2.is_intersecting(limits_),
            interval4.is_intersecting(limits_)};
        if (intersects[0] && intersects[1] && intersects[2] && intersects[3])
        {
            // print something
            return false;
        }
        lengths = {
            lengths[0] + 2 * M_PI * intersects[0],
            lengths[1] + 2 * M_PI * intersects[1],
            lengths[2] + 2 * M_PI * intersects[2],
            lengths[3] + 2 * M_PI * intersects[3]}; //punished with intersection
        if (triple_point)
        {  
             lengths.push_back(interval5.len() + 2 * M_PI * interval5.is_intersecting(limits_));
             lengths.push_back(interval6.len() + 2 * M_PI * interval6.is_intersecting(limits_));
        }

        min_arg = std::distance(lengths.begin(), std::min_element(lengths.begin(), lengths.end()));        
        if (min_arg < 2)
        {
            omega_direc_ = 1;
            this->steering_angle_ = Target;
            return true;
        }
        if (min_arg < 4)
        {
            omega_direc_ = -1;
            this->steering_angle_ = supplementary;
            return true;
        }
        else
        {
            omega_direc_ = -1;
            this->steering_angle_ = supplementary2;
            return true;
        }
    }
}

void Wheel::set_current_angle(double Angle)
{
    theta_ = utils::theta_map(Angle);
}

double Wheel::get_current_angle() const
{
    return theta_;
}

double Wheel::get_command_angle() const
{
    return steering_angle_;
}

void Wheel::set_command_velocity(double Velocity)
{
    command_omega_ = Velocity;
}

double Wheel::get_command_velocity() const
{
    return omega_direc_ * command_omega_;
}

bool Wheel::get_limitless() const
{
    return limitless_;
}

interval Wheel::get_limits() const
{
    return limits_;
}

void Wheel::set_limitless(bool Limitless)
{
    limitless_ = Limitless;
}

int Wheel::get_omega_direction() const
{
    return omega_direc_;
}
