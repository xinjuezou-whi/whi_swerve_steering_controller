/******************************************************************
odometry for swerve steering under ROS 2
it is a controller resource layer for ros2_controller

Features:
- swerve steering odometry
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-07-11: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <rclcpp/time.hpp>

#include <cmath>

namespace whi_swerve_steering_controller
{
    class Odometry
    {
    public:
        explicit Odometry(size_t VelocityRollingWindowSize = 10);

        void init(const rclcpp::Time& Time, double InfinityTolerance, double IntersectionTolerance);
        bool update(std::vector<double> WheelsOmega, std::vector<double> SteersTheta,
            std::vector<int> Directions, const rclcpp::Time& Time, std::array<double, 2>& IntersectionPoint);
        void resetOdometry();

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return heading_; }
        double getLinearX() const { return linear_x_; }
        double getLinearY() const { return linear_y_; }
        double getAngular() const { return angular_; }

        void setWheelsParams(std::vector<double>& Radii, std::vector<std::array<double, 2>>& Positions);
        void setVelocityRollingWindowSize(size_t VelocityRollingWindowSize);

    private:
        std::array<double, 2> integrateExact(double LinearRobotX, double LinearRobotY, double AngularRobot, const double Dt);
        void resetAccumulators();

        // Current timestamp:
        rclcpp::Time timestamp_{ 0 };

        size_t wheels_num_{ 0 };

        // Current pose:
        double x_{ 0.0 };        //   [m]
        double y_{ 0.0 };        //   [m]
        double heading_{ 0.0 };  // [rad]

        // Current velocity:
        double linear_x_{ 0.0 }; //   [m/s]
        double linear_y_{ 0.0 }; //   [m/s]
        double angular_{ 0.0 };  // [rad/s]

        double inf_tol_{ 1000 };
        double intersection_tol_{ 0.1 };

        // Wheel kinematic parameters [m]:
        double left_wheel_radius_;
        double right_wheel_radius_;
        std::vector<double> wheels_radii_;
        std::vector<std::array<double,2>> steers_positions_;

        // Previous wheel position/state [rad]:
        std::vector<double> wheels_old_pomega_;
        std::vector<double> steers_old_theta_;

        // Rolling mean accumulators for the linear and angular velocities:
        size_t velocity_rolling_window_size_{ 10 };
        using RollingMeanAcc = boost::accumulators::accumulator_set<double,
            boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>;
        RollingMeanAcc linear_x_accumulator_;
        RollingMeanAcc linear_y_accumulator_;
        RollingMeanAcc angular_accumulator_;
    };
}  // namespace whi_swerve_steering_controller
