/******************************************************************
swerve steering controller plugin under ROS 2
it is a controller resource layer for ros2_controller

Features:
- swerve steering kinematics and odometry
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_swerve_steering_controller/whi_swerve_steering_controller.hpp"

#include <cmath>

namespace whi_swerve_steering_controller
{
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;

    WhiSwerveSteeringController::WhiSwerveSteeringController()
        : controller_interface::ControllerInterface() {}

    controller_interface::return_type WhiSwerveSteeringController::init(const std::string& ControllerName)
    {
        // initialize lifecycle node
        auto ret = ControllerInterface::init(ControllerName);
        if (ret != controller_interface::return_type::OK)
        {
            return ret;
        }

        try
        {
            auto_declare<double>("publish_rate", publish_rate_);

            // with the lifecycle node being initialized, we can declare parameters
            auto_declare<std::vector<std::string>>("left_wheel_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("right_wheel_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("left_steer_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("right_steer_names", std::vector<std::string>());

            auto_declare<std::vector<double>>("left_wheel_radius", std::vector<double>());
            auto_declare<std::vector<double>>("left_wheel_radius_multiplier", std::vector<double>());
            auto_declare<std::vector<double>>("right_wheel_radius", std::vector<double>());
            auto_declare<std::vector<double>>("right_wheel_radius_multiplier", std::vector<double>());

            auto_declare<double>("wheel_base", wheel_base_);
            auto_declare<double>("wheel_base_multiplier", wheel_base_multiplier_);
            auto_declare<double>("track_width", track_width_);
            auto_declare<double>("track_width_multiplier", track_width_multiplier_);

            auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id_);
            auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id_);
            auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
            auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
            auto_declare<bool>("open_loop", odom_params_.open_loop_);
            auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf_);

            auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
            auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
            auto_declare<int>("velocity_rolling_window_size", 10);
            auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

            auto_declare<bool>("linear.x.has_velocity_limits", false);
            auto_declare<bool>("linear.x.has_acceleration_limits", false);
            auto_declare<bool>("linear.x.has_jerk_limits", false);
            auto_declare<double>("linear.x.max_velocity", NAN);
            auto_declare<double>("linear.x.min_velocity", NAN);
            auto_declare<double>("linear.x.max_acceleration", NAN);
            auto_declare<double>("linear.x.min_acceleration", NAN);
            auto_declare<double>("linear.x.max_jerk", NAN);
            auto_declare<double>("linear.x.min_jerk", NAN);

            auto_declare<bool>("angular.z.has_velocity_limits", false);
            auto_declare<bool>("angular.z.has_acceleration_limits", false);
            auto_declare<bool>("angular.z.has_jerk_limits", false);
            auto_declare<double>("angular.z.max_velocity", NAN);
            auto_declare<double>("angular.z.min_velocity", NAN);
            auto_declare<double>("angular.z.max_acceleration", NAN);
            auto_declare<double>("angular.z.min_acceleration", NAN);
            auto_declare<double>("angular.z.max_jerk", NAN);
            auto_declare<double>("angular.z.min_jerk", NAN);
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(node_->get_logger(),
                "\033[1;31mexception thrown during init stage with message: %s\033[0m", e.what());
            return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }

    InterfaceConfiguration WhiSwerveSteeringController::command_interface_configuration() const
    {
        std::vector<std::string> confNames;

        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    InterfaceConfiguration WhiSwerveSteeringController::state_interface_configuration() const
    {
        std::vector<std::string> confNames;

        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    controller_interface::return_type WhiSwerveSteeringController::update()
    {
        return controller_interface::return_type::OK;
    }

    CallbackReturn WhiSwerveSteeringController::on_configure(const rclcpp_lifecycle::State&)
    {
        previous_update_timestamp_ = node_->get_clock()->now();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn WhiSwerveSteeringController::on_activate(const rclcpp_lifecycle::State&)
    {
        subscriber_is_active_ = true;

        RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn WhiSwerveSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn WhiSwerveSteeringController::on_cleanup(const rclcpp_lifecycle::State&)
    {
        received_velocity_msg_ptr_.set(std::make_shared<Twist>());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn WhiSwerveSteeringController::on_error(const rclcpp_lifecycle::State&)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn WhiSwerveSteeringController::on_shutdown(const rclcpp_lifecycle::State&)
    {
        return CallbackReturn::SUCCESS;
    }
}  // namespace whi_swerve_steering_controller

#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(
    whi_swerve_steering_controller::WhiSwerveSteeringController, controller_interface::ControllerInterface)
