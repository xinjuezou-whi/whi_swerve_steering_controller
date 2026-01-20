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
#include "whi_swerve_steering_controller/speed_limiter.hpp"
#include "whi_swerve_steering_controller/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <queue>

namespace whi_swerve_steering_controller
{
    class WhiSwerveSteeringController : public controller_interface::ChainableControllerInterface
    {
        using Twist = geometry_msgs::msg::TwistStamped;

    public:
        WhiSwerveSteeringController();
        virtual ~WhiSwerveSteeringController() = default;

    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // Chainable controller replaces update() with the following two functions
        controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;

        controller_interface::return_type update_and_write_commands(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& PreState) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& PreState) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& PreState) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &PreState) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &PreState) override;

    protected:
        bool on_set_chained_mode(bool ChainedMode) override;
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        bool reset();
        void halt();
        bool getWheelsParam();

        struct WheelHandle
        {
            std::optional<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> velocity_sta_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_cmd_;
        };
        CallbackReturn configureWheelSide(const std::string& Side, const std::vector<std::string>& WheelNames,
            std::vector<WheelHandle>& RegisteredHandles);
        struct SteerHandle
        {
            std::optional<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> position_sta_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_cmd_;
        };
        CallbackReturn configureSteerSide(const std::string& Side, const std::vector<std::string>& SteerNames,
            std::vector<SteerHandle>& RegisteredHandles);

    private:
        void reset_buffers();

    protected:
        std::vector<WheelHandle> registered_left_wheel_handles_;
        std::vector<WheelHandle> registered_right_wheel_handles_;
        std::vector<SteerHandle> registered_left_steer_handles_;
        std::vector<SteerHandle> registered_right_steer_handles_;

        struct Wheel
        {
            double radius_{ 0.1 };
            std::array<double, 2> position_{ 0.0, 0.0 };
        };

        std::vector<Wheel> wheels_;
        std::vector<std::string> left_wheel_names_;
        std::vector<std::string> right_wheel_names_;
        std::vector<std::string> left_steer_names_;
        std::vector<std::string> right_steer_names_;
        double wheel_base_{ 1.0 };
        double wheel_base_multiplier_{ 1.0 };
        double track_width_{ 1.0 };
        double track_width_multiplier_{ 1.0 };

        struct OdometryParams
        {
            bool enable_odom_{ true };
            bool enable_odom_tf_{ true };
            std::string base_frame_id_{ "base_link" };
            std::string odom_frame_id_{ "odom" };
            std::array<double, 6> pose_covariance_diagonal_;
            std::array<double, 6> twist_covariance_diagonal_;
        } odom_params_;

        Odometry odometry_;

        // Timeout to consider cmd_vel commands old
        rclcpp::Duration cmd_vel_timeout_{ rclcpp::Duration::from_seconds(0.5) };

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_{ nullptr };
        nav_msgs::msg::Odometry odometry_message_;

        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
            realtime_odometry_transform_publisher_{ nullptr };
        tf2_msgs::msg::TFMessage odometry_transform_message_;

        bool subscriber_is_active_{ false };
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_{ nullptr };
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_{ nullptr };

        // the realtime container to exchange the reference from subscriber
        realtime_tools::RealtimeThreadSafeBox<Twist> received_velocity_msg_;
        // save the last reference in case of unable to get value from box
        Twist command_msg_;

        // last two commands
        std::queue<Twist> previous_two_commands_;

        // speed limiters
        std::unique_ptr<SpeedLimiter> limiter_linear_x_;
        std::unique_ptr<SpeedLimiter> limiter_linear_y_;
        std::unique_ptr<SpeedLimiter> limiter_angular_;

        bool publish_limited_velocity_{ false };
        std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_{ nullptr} ;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> unstamped_limited_velocity_publisher_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> realtime_unstamped_limited_velocity_publisher_{ nullptr} ;
        Twist limited_velocity_message_;

        rclcpp::Time previous_update_timestamp_{ 0 };

        // publish rate limiter
        double publish_rate_{ 50.0 };
        rclcpp::Duration publish_period_{ rclcpp::Duration::from_nanoseconds(0) };
        rclcpp::Time previous_publish_timestamp_{ 0, 0, RCL_CLOCK_UNINITIALIZED };

        // command values
        std::vector<double> wheels_angular_;
        std::vector<double> steers_angle_;
        std::vector<double> pre_steers_angle_;
        std::vector<double> stable_steers_angle_;

        bool use_stamped_vel_{ false };

    };
}  // namespace whi_swerve_steering_controller
