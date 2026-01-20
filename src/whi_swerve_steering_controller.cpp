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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <angles/angles.h>

#include <cmath>

namespace whi_swerve_steering_controller
{
    template <typename T> int signOf(T Val)
    {
        return (T(0) < Val) - (Val < T(0));
    }

    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;

    WhiSwerveSteeringController::WhiSwerveSteeringController()
        : controller_interface::ControllerInterface() {}

    InterfaceConfiguration WhiSwerveSteeringController::command_interface_configuration() const
    {
        std::vector<std::string> confNames;
        for (const auto& name : left_wheel_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto& name : right_wheel_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto& name : left_steer_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }
        for (const auto& name : right_steer_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }
        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    InterfaceConfiguration WhiSwerveSteeringController::state_interface_configuration() const
    {
        std::vector<std::string> confNames;
        for (const auto& name : left_wheel_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto& name : right_wheel_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto& name : left_steer_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }
        for (const auto& name : right_steer_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }

        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    controller_interface::return_type WhiSwerveSteeringController::update(const rclcpp::Time& Time,
        const rclcpp::Duration& Period)
    {
        auto logger = get_node()->get_logger();

        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<Twist> lastMsg;
        received_velocity_msg_ptr_.get(lastMsg);
        if (lastMsg == nullptr)
        {
            RCLCPP_WARN(logger, "velocity message received was a nullptr");
            return controller_interface::return_type::ERROR;
        }

        const auto dt = Time - lastMsg->header.stamp;
        // brake if cmd_vel has timeout, override the stored command
        if (dt > cmd_vel_timeout_)
        {
            lastMsg->twist.linear.x = 0.0;
            lastMsg->twist.linear.y = 0.0;
            lastMsg->twist.angular.z = 0.0;
        }

        // command may be limited further by SpeedLimit, without affecting the stored twist command
        Twist command = *lastMsg;
        double& linearCommandX = command.twist.linear.x;
        double& linearCommandY = command.twist.linear.y;
        double& angularCommand = command.twist.angular.z;

        std::vector<double> wheelsAngular, steersAngle;
        for (size_t i = 0; i < left_wheel_names_.size(); ++i)
        {
            const double angular = registered_left_wheel_handles_[i].velocity_sta_.get().get_value();
            if (std::isnan(angular))
            {
                RCLCPP_ERROR(logger, "wheel angular is invalid for index [%zu] on left side", i);
                return controller_interface::return_type::ERROR;
            }
            wheelsAngular.push_back(angular);
        }
        for (size_t i = 0; i < right_wheel_names_.size(); ++i)
        {
            const double angular = registered_right_wheel_handles_[i].velocity_sta_.get().get_value();
            if (std::isnan(angular))
            {
                RCLCPP_ERROR(logger, "wheel angular is invalid for index [%zu] on right side", i);
                return controller_interface::return_type::ERROR;
            }
            wheelsAngular.push_back(angular);
        }
        for (size_t i = 0; i < left_steer_names_.size(); ++i)
        {
            double angle = registered_left_steer_handles_[i].position_sta_.get().get_value();
            if (std::isnan(angle))
            {
                RCLCPP_ERROR(logger, "steer angle is invalid for index [%zu] on left side", i);
                return controller_interface::return_type::ERROR;
            }
            steersAngle.push_back(angle);
#ifdef DEBUG
            std::cout << "angle of " << left_steer_names_[i] << ": " << angle << std::endl;
#endif
        }
        for (size_t i = 0; i < right_steer_names_.size(); ++i)
        {
            double angle = registered_right_steer_handles_[i].position_sta_.get().get_value();
            if (std::isnan(angle))
            {
                RCLCPP_ERROR(logger, "steer angle is invalid for index [%zu] on right side", i + left_steer_names_.size());
                return controller_interface::return_type::ERROR;
            }
            steersAngle.push_back(angle);
#ifdef DEBUG
            std::cout << "angle of " << right_steer_names_[i] << ": " << angle << std::endl;
#endif
        }

        odometry_.update(wheelsAngular, steersAngle, Time);

        // publish odometry message
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, odometry_.getHeading());

        if (previous_publish_timestamp_ < Time)
        {
            previous_publish_timestamp_ += publish_period_;

            if (odom_params_.enable_odom_ && realtime_odometry_publisher_->trylock())
            {
                auto & odometry_message = realtime_odometry_publisher_->msg_;
                odometry_message.header.stamp = Time;
                odometry_message.pose.pose.position.x = odometry_.getX();
                odometry_message.pose.pose.position.y = odometry_.getY();
                odometry_message.pose.pose.orientation.x = orientation.x();
                odometry_message.pose.pose.orientation.y = orientation.y();
                odometry_message.pose.pose.orientation.z = orientation.z();
                odometry_message.pose.pose.orientation.w = orientation.w();
                odometry_message.twist.twist.linear.x  = odometry_.getLinearX();
                odometry_message.twist.twist.linear.y  = odometry_.getLinearY();
                odometry_message.twist.twist.angular.z = odometry_.getAngular();
                realtime_odometry_publisher_->unlockAndPublish();
            }

            if (odom_params_.enable_odom_tf_ && realtime_odometry_transform_publisher_->trylock())
            {
                auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
                transform.header.stamp = Time;
                transform.transform.translation.x = odometry_.getX();
                transform.transform.translation.y = odometry_.getY();
                transform.transform.rotation.x = orientation.x();
                transform.transform.rotation.y = orientation.y();
                transform.transform.rotation.z = orientation.z();
                transform.transform.rotation.w = orientation.w();
                realtime_odometry_transform_publisher_->unlockAndPublish();
            }
        }

        previous_update_timestamp_ = Time;

        auto& lastCommand = previous_two_commands_.back().twist;
        auto& secondToLastCommand = previous_two_commands_.front().twist;
        limiter_linear_x_.limit(
            linearCommandX, lastCommand.linear.x, secondToLastCommand.linear.x, Period.seconds());
        limiter_linear_y_.limit(
            linearCommandY, lastCommand.linear.y, secondToLastCommand.linear.y, Period.seconds());
        limiter_angular_.limit(
            angularCommand, lastCommand.angular.z, secondToLastCommand.angular.z, Period.seconds());

        previous_two_commands_.pop();
        previous_two_commands_.emplace(command);

        // publish limited velocity
        if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
        {
            auto& limitedVelocityCommand = realtime_limited_velocity_publisher_->msg_;
            limitedVelocityCommand.header.stamp = Time;
            limitedVelocityCommand.twist = command.twist;
            realtime_limited_velocity_publisher_->unlockAndPublish();
        }

        // compute wheels velocities and steer positions and set them
        for (size_t i = 0; i < left_wheel_names_.size() + right_wheel_names_.size(); ++i)
        {
            // inverse kinematics
            double wheelLinearX = command.twist.linear.x - command.twist.angular.z * wheels_[i].position_[1];
            double wheelLinearY = command.twist.linear.y + command.twist.angular.z * wheels_[i].position_[0];
            
            // get the required wheel angular and the required steering angle 
            wheels_angular_[i]  = sqrt(pow(wheelLinearX, 2) + pow(wheelLinearY, 2)) / wheels_[i].radius_;
            steers_angle_[i] = atan2(wheelLinearY, wheelLinearX);

#ifdef DEBUG
            std::cout << "steer[" << i << "] angle: " << steerAngle << ", angular: " << wheelAngular << std::endl;
#endif
            if (fabs(steers_angle_[i]) > 0.5 * M_PI)
            {
                steers_angle_[i] -= signOf(steers_angle_[i]) * M_PI;
                wheels_angular_[i] *= -1.0;
            }
#ifdef DEBUG
            std::cout << "steer[" << i << "] normal angle: " << steers_angle_[i] << ", normal angular: " << wheels_angular_[i] << std::endl;
#endif
        }

        // detect the big change
        static bool stable = true;
        if (stable)
        {
            for (size_t i = 0; i < left_wheel_names_.size() + right_wheel_names_.size(); ++i)
            {
                if (fabs(fabs(steers_angle_[i]) - fabs(pre_steers_angle_[i])) > angles::from_degrees(40.0))
                {
                    stable = false;
                    break;
                }
            }
            if (!stable)
            {
                stable_steers_angle_ = steers_angle_;
                wheels_angular_.resize(wheels_angular_.size(), 0.0);
            }
            else
            {
                pre_steers_angle_ = steers_angle_;
            }
        }
        else
        {
            // check if the steers are stable
            stable = true;
            for (size_t i = 0; i < left_wheel_names_.size() + right_wheel_names_.size(); ++i)
            {
                if (fabs(fabs(stable_steers_angle_[i]) - fabs(steersAngle[i])) > angles::from_degrees(2.0))
                {
                    stable = false;
                    break;
                }
            }
            if (!stable)
            {
                wheels_angular_.resize(wheels_angular_.size(), 0.0);
            }
            else
            {
                pre_steers_angle_ = steers_angle_;
            }
        }
        
        for (size_t i = 0; i < left_wheel_names_.size() + right_wheel_names_.size(); ++i)
        {
            if (i < left_wheel_names_.size())
            {
                registered_left_steer_handles_[i].position_cmd_.get().set_value(steers_angle_[i]);
                registered_left_wheel_handles_[i].velocity_cmd_.get().set_value(wheels_angular_[i]);
            }
            else
            {
                registered_right_steer_handles_[i - left_wheel_names_.size()].position_cmd_.get().set_value(steers_angle_[i]);
                registered_right_wheel_handles_[i - left_wheel_names_.size()].velocity_cmd_.get().set_value(wheels_angular_[i]);
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_init()
    {
        /// node version and copyright announcement
        std::cout << "\nWHI swerve steering controller VERSION 0.3.1" << std::endl;
        std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        try
        {
            auto_declare<double>("publish_rate", publish_rate_);

            // with the lifecycle node being initialized, we can declare parameters
            auto_declare<std::vector<std::string>>("left_wheel_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("right_wheel_names", std::vector<std::string>());
            auto_declare<std::vector<double>>("left_wheel_radius", std::vector<double>());
            auto_declare<std::vector<double>>("left_wheel_radius_multiplier", std::vector<double>());
            auto_declare<std::vector<double>>("right_wheel_radius", std::vector<double>());
            auto_declare<std::vector<double>>("right_wheel_radius_multiplier", std::vector<double>());

            auto_declare<std::vector<std::string>>("left_steer_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("right_steer_names", std::vector<std::string>());
            auto_declare<std::vector<double>>("left_steer_locations", std::vector<double>());
            auto_declare<std::vector<double>>("right_steer_locations", std::vector<double>());
            auto_declare<std::vector<double>>("left_steer_limits", std::vector<double>());
            auto_declare<std::vector<double>>("right_steer_limits", std::vector<double>());

            auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id_);
            auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id_);
            auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
            auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
            auto_declare<bool>("enable_odom", odom_params_.enable_odom_);
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
            RCLCPP_FATAL(get_node()->get_logger(),
                "\033[1;31mexception thrown during init stage with message: %s\033[0m", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_configure(
        const rclcpp_lifecycle::State&)
    {
        auto logger = get_node()->get_logger();

        /// update parameters
        // wheels
        left_wheel_names_ = get_node()->get_parameter("left_wheel_names").as_string_array();
        right_wheel_names_ = get_node()->get_parameter("right_wheel_names").as_string_array();
        if (left_wheel_names_.empty() || right_wheel_names_.empty())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31meither the left wheels or the right wheels are empty\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (left_wheel_names_.size() != right_wheel_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of left wheels [%zu] and the number of right wheels [%zu] are different\033[0m",
                left_wheel_names_.size(), right_wheel_names_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        // steers
        left_steer_names_ = get_node()->get_parameter("left_steer_names").as_string_array();
        right_steer_names_ = get_node()->get_parameter("right_steer_names").as_string_array();
        if (left_steer_names_.empty() || right_steer_names_.empty())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31meither the left steers or the right steers are empty\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (left_steer_names_.size() != right_steer_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of left steers [%zu] and the number of right steers [%zu] are different\033[0m",
                left_steer_names_.size(), right_steer_names_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!getWheelsParam())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // odom
        std::vector<double> radii;
        std::vector<std::array<double, 2>> steerPositions;
        for (const auto& it: wheels_)
        {
            radii.push_back(it.radius_);
            steerPositions.push_back(it.position_);
        }

        odometry_.init(get_node()->get_clock()->now());
        odometry_.setWheelsParams(radii, steerPositions);
        odometry_.setVelocityRollingWindowSize(get_node()->get_parameter("velocity_rolling_window_size").as_int());

        odom_params_.odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
        odom_params_.base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();

        auto poseDiagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
        std::copy(poseDiagonal.begin(), poseDiagonal.end(), odom_params_.pose_covariance_diagonal_.begin());
        auto twistDiagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
        std::copy(twistDiagonal.begin(), twistDiagonal.end(), odom_params_.twist_covariance_diagonal_.begin());

        odom_params_.enable_odom_ = get_node()->get_parameter("enable_odom").as_bool();
        odom_params_.enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();

        // cmd_vel
        cmd_vel_timeout_ = std::chrono::milliseconds{
            static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
        publish_limited_velocity_ = get_node()->get_parameter("publish_limited_velocity").as_bool();
        use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

        // speed limit
        try
        {
            limiter_linear_x_ = SpeedLimiter(
                get_node()->get_parameter("linear.x.has_velocity_limits").as_bool(),
                get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool(),
                get_node()->get_parameter("linear.x.has_jerk_limits").as_bool(),
                get_node()->get_parameter("linear.x.min_velocity").as_double(),
                get_node()->get_parameter("linear.x.max_velocity").as_double(),
                get_node()->get_parameter("linear.x.min_acceleration").as_double(),
                get_node()->get_parameter("linear.x.max_acceleration").as_double(),
                get_node()->get_parameter("linear.x.min_jerk").as_double(),
                get_node()->get_parameter("linear.x.max_jerk").as_double());
            limiter_linear_y_ = SpeedLimiter(
                get_node()->get_parameter("linear.y.has_velocity_limits").as_bool(),
                get_node()->get_parameter("linear.y.has_acceleration_limits").as_bool(),
                get_node()->get_parameter("linear.y.has_jerk_limits").as_bool(),
                get_node()->get_parameter("linear.y.min_velocity").as_double(),
                get_node()->get_parameter("linear.y.max_velocity").as_double(),
                get_node()->get_parameter("linear.y.min_acceleration").as_double(),
                get_node()->get_parameter("linear.y.max_acceleration").as_double(),
                get_node()->get_parameter("linear.y.min_jerk").as_double(),
                get_node()->get_parameter("linear.y.max_jerk").as_double());
        }
        catch (const std::runtime_error & e)
        {
            RCLCPP_ERROR(logger,
                "\033[1;31merror configuring linear speed limiter: %s\033[0m", e.what());
        }
        try
        {
            limiter_angular_ = SpeedLimiter(
                get_node()->get_parameter("angular.z.has_velocity_limits").as_bool(),
                get_node()->get_parameter("angular.z.has_acceleration_limits").as_bool(),
                get_node()->get_parameter("angular.z.has_jerk_limits").as_bool(),
                get_node()->get_parameter("angular.z.min_velocity").as_double(),
                get_node()->get_parameter("angular.z.max_velocity").as_double(),
                get_node()->get_parameter("angular.z.min_acceleration").as_double(),
                get_node()->get_parameter("angular.z.max_acceleration").as_double(),
                get_node()->get_parameter("angular.z.min_jerk").as_double(),
                get_node()->get_parameter("angular.z.max_jerk").as_double());
        }
        catch (const std::runtime_error & e)
        {
            RCLCPP_ERROR(logger,
                "\033[1;31merror configuring angular speed limiter: %s\033[0m", e.what());
        }

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (publish_limited_velocity_)
        {
            limited_velocity_publisher_ =
                get_node()->create_publisher<Twist>("~/cmd_vel_out", rclcpp::SystemDefaultsQoS());
            realtime_limited_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
        }
    
        const Twist emptyTwist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(emptyTwist));

        // Fill last two commands with default constructed commands
        previous_two_commands_.emplace(emptyTwist);
        previous_two_commands_.emplace(emptyTwist);

        // initialize command subscriber
        if (use_stamped_vel_)
        {
            velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
                "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
                [this, &logger](const std::shared_ptr<Twist> msg) -> void
                {
                    if (!subscriber_is_active_)
                    {
                        RCLCPP_WARN(logger, "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                    {
                        RCLCPP_WARN_ONCE(logger,
                            "received TwistStamped with zero timestamp, setting it to current "
                            "time, this message will only be shown once");

                        msg->header.stamp = get_node()->get_clock()->now();
                    }
                    received_velocity_msg_ptr_.set(std::move(msg));
                });
        }
        else
        {
            velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
                "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
                [this, &logger](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
                {
                    if (!subscriber_is_active_)
                    {
                        RCLCPP_WARN(logger, "can't accept new commands. subscriber is inactive");
                        return;
                    }

                    // Write fake header in the stored stamped command
                    std::shared_ptr<Twist> twistStamped;
                    received_velocity_msg_ptr_.get(twistStamped);
                    twistStamped->twist = *msg;
                    twistStamped->header.stamp = get_node()->get_clock()->now();
                });
        }

        // initialize odometry publisher and messasge
        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

        auto& odometryMsg = realtime_odometry_publisher_->msg_;
        odometryMsg.header.frame_id = odom_params_.odom_frame_id_;
        odometryMsg.child_frame_id = odom_params_.base_frame_id_;
        // initialize odom values zeros first
        odometryMsg.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
        const size_t NUM_DIMENSIONS = 6;
        for (size_t i = 0; i < NUM_DIMENSIONS; ++i)
        {
            // 0, 7, 14, 21, 28, 35
            const size_t diagonalIndex = NUM_DIMENSIONS * i + i;
            odometryMsg.pose.covariance[diagonalIndex] = odom_params_.pose_covariance_diagonal_[i];
            odometryMsg.twist.covariance[diagonalIndex] = odom_params_.twist_covariance_diagonal_[i];
        }

        // initialize transform publisher and message
        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::SystemDefaultsQoS());
        realtime_odometry_transform_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);

        // keeping track of odom and base_link transforms only
        auto& odometryTransformMsg = realtime_odometry_transform_publisher_->msg_;
        odometryTransformMsg.transforms.resize(1);
        odometryTransformMsg.transforms.front().header.frame_id = odom_params_.odom_frame_id_;
        odometryTransformMsg.transforms.front().child_frame_id = odom_params_.base_frame_id_;

        // limit the publication on the topics /odom and /tf
        publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
        previous_publish_timestamp_ = get_node()->get_clock()->now();

        previous_update_timestamp_ = get_node()->get_clock()->now();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_activate(
        const rclcpp_lifecycle::State&)
    {
        const auto leftWheelResult =
            configureWheelSide("left", left_wheel_names_, registered_left_wheel_handles_);
        const auto rightWheelResult =
            configureWheelSide("right", right_wheel_names_, registered_right_wheel_handles_);
        const auto leftSteerResult =
            configureSteerSide("left", left_steer_names_, registered_left_steer_handles_);
        const auto rightSteerResult =
            configureSteerSide("right", right_steer_names_, registered_right_steer_handles_);

        if (leftWheelResult == controller_interface::CallbackReturn::ERROR ||
            rightWheelResult == controller_interface::CallbackReturn::ERROR ||
            leftSteerResult == controller_interface::CallbackReturn::ERROR ||
            rightSteerResult == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty() ||
            registered_left_steer_handles_.empty() || registered_right_steer_handles_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "\033[1;31meither left wheel or steer interfaces, right wheel or steer interfaces are non existent\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted = false;
        subscriber_is_active_ = true;

        RCLCPP_INFO(get_node()->get_logger(),
            "\033[1;32msubscriber and publisher of swerve controller are now active\033[0m");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }

        registered_left_wheel_handles_.clear();
        registered_right_wheel_handles_.clear();
        registered_left_steer_handles_.clear();
        registered_right_steer_handles_.clear();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        received_velocity_msg_ptr_.set(std::make_shared<Twist>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::on_error(
        const rclcpp_lifecycle::State&)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::configureWheelSide(
        const std::string& Side, const std::vector<std::string>& WheelNames,
        std::vector<WheelHandle>& RegisteredHandles)
    {
        auto logger = get_node()->get_logger();

        if (WheelNames.empty())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        RegisteredHandles.reserve(WheelNames.size());
        for (const auto& name : WheelNames)
        {
            const auto stateHandle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto& interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });

            if (stateHandle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain wheel state handle for %s\033[0m", name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto commandHandle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&name](const auto & interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });

            if (commandHandle == command_interfaces_.end())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain wheel command handle for %s\033[0m", name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            RegisteredHandles.emplace_back(WheelHandle{std::ref(*stateHandle), std::ref(*commandHandle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiSwerveSteeringController::configureSteerSide(
        const std::string& Side, const std::vector<std::string>& SteerNames,
        std::vector<SteerHandle>& RegisteredHandles)
    {
        auto logger = get_node()->get_logger();

        if (SteerNames.empty())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        RegisteredHandles.reserve(SteerNames.size());
        for (const auto& name : SteerNames)
        {
            const auto stateHandle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto& interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });

            if (stateHandle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain steer state handle for %s\033[0m", name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto commandHandle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&name](const auto & interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });

            if (commandHandle == command_interfaces_.end())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain steer command handle for %s\033[0m", name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            RegisteredHandles.emplace_back(SteerHandle{std::ref(*stateHandle), std::ref(*commandHandle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool WhiSwerveSteeringController::reset()
    {
        odometry_.resetOdometry();

        // release the old queue
        std::queue<Twist> empty;
        std::swap(previous_two_commands_, empty);

        registered_left_wheel_handles_.clear();
        registered_right_wheel_handles_.clear();
        registered_left_steer_handles_.clear();
        registered_right_steer_handles_.clear();

        subscriber_is_active_ = false;
        velocity_command_subscriber_.reset();
        velocity_command_unstamped_subscriber_.reset();

        received_velocity_msg_ptr_.set(nullptr);
        is_halted = false;

        return true;
    }

    void WhiSwerveSteeringController::halt()
    {
        const auto haltWheels = [](auto& WheelHandles)
        {
            for (const auto& wheel : WheelHandles)
            {
                wheel.velocity_cmd_.get().set_value(0.0);
            }
        };

        haltWheels(registered_left_wheel_handles_);
        haltWheels(registered_right_wheel_handles_);

        // TBD:: do we need restore the steer to 0 position?
    }

    bool WhiSwerveSteeringController::getWheelsParam()
    {
        auto logger = get_node()->get_logger();

        // wheels
        wheels_.resize(left_wheel_names_.size() + right_wheel_names_.size());
        wheels_angular_.resize(wheels_.size(), 0.0);
        steers_angle_.resize(wheels_.size(), 0.0);
        pre_steers_angle_.resize(wheels_.size(), 0.0);
        stable_steers_angle_.resize(wheels_.size(), 0.0);
        auto radiusLeft = get_node()->get_parameter("left_wheel_radius").as_double_array();
        auto radiusMultiLeft = get_node()->get_parameter("left_wheel_radius_multiplier").as_double_array();
        if (radiusLeft.size() != left_wheel_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of left wheels radius [%zu] and the number of left wheels [%zu] are different\033[0m",
                radiusLeft.size(), left_wheel_names_.size());

            return false;
        }
        if (radiusLeft.size() > radiusMultiLeft.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of left wheels radius [%zu] and the number of left wheels radius multiplier [%zu] are different\033[0m",
                radiusLeft.size(), radiusMultiLeft.size());

            return false;
        }
        auto radiusRight = get_node()->get_parameter("right_wheel_radius").as_double_array();
        auto radiusMultiRight = get_node()->get_parameter("right_wheel_radius_multiplier").as_double_array();
        if (radiusRight.size() != right_wheel_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of right wheels radius [%zu] and the number of right wheels [%zu] are different\033[0m",
                radiusRight.size(), right_wheel_names_.size());

            return false;
        }
        if (radiusRight.size() > radiusMultiRight.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of right wheels radius [%zu] and the number of right wheels radius multiplier [%zu] are different\033[0m",
                radiusLeft.size(), radiusMultiLeft.size());

            return false;
        }

        int leftSideSize = left_steer_names_.size();
        for (auto i = 0; i < leftSideSize + right_wheel_names_.size(); ++i)
        {
            wheels_[i].radius_ = i < leftSideSize ? radiusLeft[i] * radiusMultiLeft[i] :
                radiusRight[i - leftSideSize] * radiusMultiRight[i - leftSideSize];
#ifdef DEBUG
            std::cout << "index " << i << " radius: " << wheels_[i].radius_ << std::endl;
#endif
        }

        // steers
        auto flatenPosLeft = get_node()->get_parameter("left_steer_locations").as_double_array();
        if (flatenPosLeft.size() / 2 != left_steer_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of left steer locations [%zu] and the number of left steers [%zu] are different\033[0m",
                flatenPosLeft.size() / 2, left_steer_names_.size());

            return false;
        }
        std::vector<std::array<double, 2>> leftPositions;
        for (auto i = 0; i < flatenPosLeft.size() / 2; ++i)
        {
            leftPositions.push_back(std::array<double, 2>{flatenPosLeft[i * 2], flatenPosLeft[i * 2 + 1]});
        }

        auto flatenPosRight = get_node()->get_parameter("right_steer_locations").as_double_array();
        if (flatenPosRight.size() / 2 != right_steer_names_.size())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31mthe number of right steer locations [%zu] and the number of right steers [%zu] are different\033[0m",
                flatenPosRight.size() / 2, right_steer_names_.size());

            return false;
        }
        std::vector<std::array<double, 2>> rightPositions;
        for (auto i = 0; i < flatenPosRight.size() / 2; ++i)
        {
            rightPositions.push_back(std::array<double, 2>{flatenPosRight[i * 2], flatenPosRight[i * 2 + 1]});
        }

        for (auto i = 0; i < leftSideSize + right_steer_names_.size(); ++i)
        {
            wheels_[i].position_ = i < leftSideSize ? leftPositions[i] : rightPositions[i - leftSideSize];
#ifdef DEBUG
            std::cout << "index " << i << ", position: " << wheels_[i].position_[0] << "," <<
                wheels_[i].position_[1] << "; offset: " << wheels_[i].offset_ << std::endl;
#endif
        }

        return true;
    }
}  // namespace whi_swerve_steering_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_swerve_steering_controller::WhiSwerveSteeringController,
    controller_interface::ControllerInterface)
