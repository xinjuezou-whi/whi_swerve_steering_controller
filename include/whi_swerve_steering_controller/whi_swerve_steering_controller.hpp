/******************************************************************
swerve steering controller plugin under ROS 2
it is a controller resource layer for ros2_controller

Features:
- swerve steering kinematics and odometry
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-07-10: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_swerve_steering_controller/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace whi_swerve_steering_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class WhiSwerveSteeringController : public controller_interface::ControllerInterface
    {
        using Twist = geometry_msgs::msg::TwistStamped;

    public:
        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        WhiSwerveSteeringController();

        virtual ~WhiSwerveSteeringController() = default;

    public:
        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        controller_interface::return_type init(const std::string & controller_name) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        controller_interface::return_type update() override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        WHI_SWERVE_STEERING_CONTROLLER_PUBLIC
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    protected:
        double wheel_base_{ 1.0 };
        double wheel_base_multiplier_{ 1.0 };
        double track_width_{ 1.0 };
        double track_width_multiplier_{ 1.0 };
        struct OdometryParams
        {
            bool open_loop_{ false };
            bool enable_odom_tf_{ true };
            std::string base_frame_id_{ "base_link" };
            std::string odom_frame_id_{ "odom" };
            std::array<double, 6> pose_covariance_diagonal_;
            std::array<double, 6> twist_covariance_diagonal_;
        } odom_params_;

        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        bool subscriber_is_active_{ false };
        rclcpp::Time previous_update_timestamp_{ 0 };

        std::chrono::milliseconds cmd_vel_timeout_{ 500 };
        bool use_stamped_vel_{ false };

        bool publish_limited_velocity_{ false };
        std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_{ nullptr} ;

        // publish rate limiter
        double publish_rate_{ 50.0 };
        rclcpp::Duration publish_period_{ 0, 0 };
        rclcpp::Time previous_publish_timestamp_{ 0 };
    };
}  // namespace whi_swerve_steering_controller
