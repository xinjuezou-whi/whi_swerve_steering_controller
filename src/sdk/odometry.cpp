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
#include "whi_swerve_steering_controller/utils.h"

namespace whi_swerve_steering_controller
{
    Odometry::Odometry(size_t VelocityRollingWindowSize)
        : velocity_rolling_window_size_(VelocityRollingWindowSize),
          linear_x_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_),
          linear_y_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_),
          angular_accumulator_(boost::accumulators::tag::rolling_window::window_size = velocity_rolling_window_size_)
    {}

    void Odometry::init(const rclcpp::Time& Time, double InfinityTolerance, double IntersectionTolerance)
    {
        // Reset accumulators and timestamp:
        resetAccumulators();
        timestamp_ = Time;
        inf_tol_  = InfinityTolerance;
        intersection_tol_ = IntersectionTolerance;
    }

    bool Odometry::update(std::vector<double> WheelsOmega, std::vector<double> SteersTheta,
        std::vector<int> Directions, const rclcpp::Time& Time, std::array<double, 2>& IntersectionPoint)
    {
        // We cannot estimate the speed with very small time intervals:
        const double dt = Time.seconds() - timestamp_.seconds();
        if (dt < 0.0001)
        {
            return false;  // Interval too small to integrate with
        }

        for (size_t i = 0; i < wheels_num_; ++i)
        {
            WheelsOmega[i] = fabs(WheelsOmega[i]);
            if (Directions[i] < 0) // i think this will make a problem when actual omega is too +ve big and the command omega is -ve 
            {                    // as it will take time to reverse and signal will be wrong all this
                SteersTheta[i] = utils::theta_map(SteersTheta[i] + M_PI);
            }
        }

        //intersection point 
        double theta1, m1, b1, theta, m, b;
        std::vector<std::array<double, 2>> intersections;
        for (size_t j = 0; j < wheels_num_ - 1; ++j)
        {
            theta1 = utils::theta_map(SteersTheta[j] + M_PI_2);
            m1 = tan(theta1);
            b1 = steers_positions_[j][1]- m1 * steers_positions_[j][0];
            for (size_t i = j + 1; i < wheels_num_; ++i)
            {
                theta = utils::theta_map(SteersTheta[i] + M_PI_2);
                m = tan(theta);
                b = steers_positions_[i][1] - m * steers_positions_[i][0];

                if (utils::isclose(theta1, theta) || utils::isclose(theta1, theta, 0.00001, 2 * M_PI))
                {
                    intersections.push_back({INFINITY,INFINITY});
                }
                else if (utils::isclose(theta1, theta, 0.00001, M_PI) || utils::isclose(theta1, theta, 0.00001, -M_PI))
                {
                    intersections.push_back({(steers_positions_[j][0] + steers_positions_[i][0]) / 2,
                        (steers_positions_[j][1] + steers_positions_[i][1]) / 2});
                }
                else
                {
                    if (fabs(((b1 - b) / (m - m1))) > inf_tol_ || fabs(((m * b1 - m1 * b) / (m - m1))) > inf_tol_)
                    {
                        intersections.push_back({INFINITY, INFINITY});
                    }
                    else
                    {
                        intersections.push_back({((b1 - b) / (m - m1)), ((m * b1 - m1 * b) / (m - m1))});
                    }
                }
            }
        }

        double linear_x, linear_x_vh, linear_y, linear_y_vh, angular;
        std::array<double, 2> average_intersection{0, 0};

        // detecting if all or some of the intersections is inf
        bool inf_all = true;
        for (const auto& it: intersections)
        {
            if (!(std::isinf(it[0]) || std::isinf(it[1])))
            {
                inf_all = false;
                break;
            }
        }
        if (inf_all)
        {
            angular = 0;
            for (int i = 0; i < wheels_num_; ++i)
            {
                linear_x_vh += (WheelsOmega[i] * wheels_radii_[i] * cos(SteersTheta[i])) / wheels_num_;
                linear_y_vh += (WheelsOmega[i] * wheels_radii_[i] * sin(SteersTheta[i])) / wheels_num_;
                IntersectionPoint[0] = INFINITY; //just to visualize it on rqt_plot through the publisher
                IntersectionPoint[1] = INFINITY; //just to visualize it on rqt_plot through the publisher
            }
        }
        else
        {
            for (size_t i = 0; i < intersections.size(); ++i)
            {
                if ((fabs(intersections[i][0] - intersections[i-1][0]) > intersection_tol_ ||
                    fabs(intersections[i][1] - intersections[i-1][1]) > intersection_tol_) && i != 0)
                {
                    // intersections are not close enough to get an average, dropping
                    return false;
                }
                else
                {
                    average_intersection[0] += intersections[i][0] / intersections.size();
                    average_intersection[1] += intersections[i][1] / intersections.size();
                }
            }
            IntersectionPoint[0] = average_intersection[0]; //just to visualize it on rqt_plot through the publisher
            IntersectionPoint[1] = average_intersection[1]; //just to visualize it on rqt_plot through the publisher
            for (int i = 0; i < wheels_num_; ++i)
            {
                // ignore the wheel if the intersection is on its center of rotation
                if (utils::isclose(average_intersection[0], steers_positions_[i][0]) &&
                    utils::isclose(average_intersection[1], steers_positions_[i][1]))
                {
                    continue;
                }
                auto icr_wh = std::array<double, 2>{steers_positions_[i][0] - average_intersection[0],
                    steers_positions_[i][1] - average_intersection[1]};

                angular += (WheelsOmega[i] * wheels_radii_[i] * sin(SteersTheta[i])) / (2 * icr_wh[0]) -
                    (WheelsOmega[i]  *wheels_radii_[i] * cos(SteersTheta[i])) / (2 * icr_wh[1]);
            }
            angular /= wheels_num_; 
            linear_x_vh = average_intersection[1] * angular;
            linear_y_vh = -1 * average_intersection[0] * angular;
        }

        if (std::isnan(linear_x_vh) || std::isnan(linear_y_vh) || std::isnan(angular))
        {
            return false;
        }
        if (std::isinf(linear_x_vh) || std::isinf(linear_y_vh) || std::isinf(angular))
        {
            return false;
        }

#ifdef DEBUG
        std::cout << "heading: " << heading_ << std::endl;
        std::cout << "linear_x_vh: " << linear_x_vh << ", linear_y_vh: " << linear_y_vh << std::endl;
        std::cout << "linear_x: " << linear_x_ << ", linear_y:" << linear_y_ << ", angular: " << angular_ << std::endl; 
#endif

        // Integrate odometry:
        auto linearWord = integrateExact(linear_x_vh, linear_y_vh, angular, dt);

        timestamp_ = Time;

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

    std::array<double, 2> Odometry::integrateExact(double LinearRobotX, double LinearRobotY, double AngularRobot, const double Dt)
    {
        /// Runge-Kutta 2nd order integration:
        double linearX = LinearRobotX * cos(heading_) - LinearRobotY * sin(heading_);
        double linearY = LinearRobotX * sin(heading_) + LinearRobotY * cos(heading_);

        x_ += linearX * Dt;
        y_ += linearY * Dt;
        heading_ += AngularRobot * Dt;

        return std::array<double, 2>{linearX, linearY};
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
