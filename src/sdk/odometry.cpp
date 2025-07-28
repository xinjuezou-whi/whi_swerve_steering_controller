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

/******************************************************************
odometry for swerve steering under ROS 2
it is a controller resource layer for ros2_controller

Features:
- swerve steering odometry
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_swerve_steering_controller/odometry.hpp"

#include <iostream>

namespace whi_swerve_steering_controller
{
    Odometry::Odometry(size_t VelocityRollingWindowSize)
        : velocity_rolling_window_size_(VelocityRollingWindowSize),
          linear_x_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_),
          linear_y_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_),
          angular_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_)
    {}

    void Odometry::init(const rclcpp::Time& Time)
    {
        // Reset accumulators and timestamp:
        resetAccumulators();
        timestamp_ = Time;
    }

    bool Odometry::update(std::vector<double> WheelAngular, std::vector<double> SteersAngle, const rclcpp::Time& Time)
    {
        // We cannot estimate the speed with very small time intervals:
        const double dt = Time.seconds() - timestamp_.seconds();
        timestamp_ = Time;
        if (dt < 0.0001)
        {
            return false;  // Interval too small to integrate with
        }

        double linearXSum = 0.0, linearYSum = 0.0, numerator = 0.0, denominator = 0.0;
        for (size_t i = 0; i < wheels_num_; ++i)
        {
            auto wheelLinearX = WheelAngular[i] * wheels_radii_[i] * cos(SteersAngle[i]);
            auto wheelLinearY = WheelAngular[i] * wheels_radii_[i] * sin(SteersAngle[i]);
            linearXSum += wheelLinearX;
            linearYSum += wheelLinearY;
            numerator += wheelLinearY * steers_positions_[i][0] - wheelLinearX * steers_positions_[i][1];
            denominator += pow(steers_positions_[i][0], 2) + pow(steers_positions_[i][1], 2);
        }
        double linearX = linearXSum / wheels_num_;
        double linearY = linearYSum / wheels_num_;
        double angular = numerator / denominator;

        auto linearWord = integrateExact(linearX, linearY, angular, dt);

        // Estimate speeds using a rolling mean to filter them out:
        linear_x_accumulator_(linearWord[0]);
        linear_y_accumulator_(linearWord[1]);
        angular_accumulator_(angular);

        linear_x_ = boost::accumulators::rolling_mean(linear_x_accumulator_);
        linear_y_ = boost::accumulators::rolling_mean(linear_y_accumulator_);
        angular_ = boost::accumulators::rolling_mean(angular_accumulator_);

        return true;
    }

    void Odometry::resetOdometry()
    {
        x_ = 0.0;
        y_ = 0.0;
        heading_ = 0.0;
    }

    void Odometry::setWheelsParams(std::vector<double>& Radii, std::vector<std::array<double, 2>>& Positions)
    {
        wheels_radii_ =  std::move(Radii);
        steers_positions_ = std::move(Positions);
        wheels_num_ = wheels_radii_.size();
    }

    void Odometry::setVelocityRollingWindowSize(size_t VelocityRollingWindowSize)
    {
        velocity_rolling_window_size_ = VelocityRollingWindowSize;
        resetAccumulators();
    }

    std::array<double, 2> Odometry::integrateExact(double LinearX, double LinearY, double Angular, const double Dt)
    {
        heading_ += Angular * Dt;

        double linearGlobalX = LinearX * cos(heading_) - LinearY * sin(heading_);
        double linearGlobalY = LinearX * sin(heading_) + LinearY * cos(heading_);
        x_ += linearGlobalX * Dt;
        y_ += linearGlobalY * Dt;
        
#ifdef DEBUG
        std::cout << "linearRobotX:" << LinearX << ", linearRobotY: " << LinearY << ", angularRobot: " << Angular << std::endl;
        std::cout << "LinearGlobalX: " << linearGlobalX << ", linearGlobalY: " << linearGlobalY <<
            "heading: " << heading_ << std::endl;
        std::cout << "x: " << x_ << ", y: " << y_ << std::endl;
#endif

        return std::array<double, 2>{linearGlobalX, linearGlobalY};
    }

    void Odometry::resetAccumulators()
    {
        linear_x_accumulator_ = RollingMeanAcc(boost::accumulators::tag::rolling_window::window_size =
            velocity_rolling_window_size_);
        linear_y_accumulator_ = RollingMeanAcc(boost::accumulators::tag::rolling_window::window_size =
            velocity_rolling_window_size_);
        angular_accumulator_  = RollingMeanAcc(boost::accumulators::tag::rolling_window::window_size =
            velocity_rolling_window_size_);
    }
}  // namespace whi_swerve_steering_controller
