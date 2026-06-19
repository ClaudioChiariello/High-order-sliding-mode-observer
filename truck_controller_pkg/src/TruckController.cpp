#include "TruckController.h"
#include <limits>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace{
    constexpr auto DEFAULT_COMMAND_TOPIC = "cmd_vel";

    using ControllerTwistReferenceMsg = truck_controller::TruckController::ControllerTwistReferenceMsg;

    void reset_controller_reference_msg(ControllerTwistReferenceMsg & msg, const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
    {
        msg.header.stamp = node->now();
        
        msg.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        msg.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
        msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
        msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
        msg.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    }
}


namespace truck_controller
{
    TruckController::TruckController(): controller_interface::ControllerInterface() {}
    
    controller_interface::CallbackReturn TruckController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "Initializing TruckController");
        // Initialize parameters

        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }

        catch(const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.full_traction)
            set_interface_numbers(12, 12);
        else
            set_interface_numbers(8, 8);
        
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TruckController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto logger = get_node()->get_logger();

        // update parameters if they have changed
        if (param_listener_->try_update_params(params_))
            RCLCPP_INFO(logger, "Parameters were updated");

        if ((params_.traction_joints_names.size() != 8 && params_.full_traction) || (params_.traction_joints_names.size() != 4 && !params_.full_traction))
        {
            RCLCPP_ERROR(logger, "Bicycle configuration requires exactly one traction joint, but %zu were provided", params_.traction_joints_names.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.steering_joints_names.size() != 4)
        {
            RCLCPP_ERROR(logger, "Ackermann configuration requires exactly two steering joints, but %zu were provided", params_.steering_joints_names.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!params_.traction_joints_state_names.empty())
        {
            if (params_.traction_joints_state_names.size() != 4 && !params_.full_traction)
            {
                RCLCPP_ERROR(logger, "Truck configuration requires exactly four traction joints, but %zu state interface names were provided",
                params_.traction_joints_state_names.size());
                return controller_interface::CallbackReturn::ERROR;
            }

            if (params_.traction_joints_state_names.size() != 8 && params_.full_traction)
            {
                RCLCPP_ERROR(logger, "Truck configuration requires exactly eight traction joints, but %zu state interface names were provided",
                params_.traction_joints_state_names.size());
                return controller_interface::CallbackReturn::ERROR;
            }

            this->traction_joints_state_names_ = params_.traction_joints_state_names;
        }

        else
            this->traction_joints_state_names_ = params_.traction_joints_names;

        if (!params_.steering_joints_state_names.empty())
        {
            if (params_.steering_joints_state_names.size() != 4)
            {
                RCLCPP_ERROR(logger, "Truck configuration requires exactly four steering joints, but %zu state interface names were provided",
                params_.steering_joints_state_names.size());
                return controller_interface::CallbackReturn::ERROR;
            }

            this->steering_joints_state_names_ = params_.steering_joints_state_names;
        }

        else
            this->steering_joints_state_names_ = params_.steering_joints_names;

        auto subscribers_qos = rclcpp::SystemDefaultsQoS();
        subscribers_qos.keep_last(1);
        subscribers_qos.best_effort();

        if(!reset())
            return controller_interface::CallbackReturn::ERROR;

        // Reference Subscriber
        ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
        ref_subscriber_twist_ = get_node()->create_subscription<ControllerTwistReferenceMsg>(
            DEFAULT_COMMAND_TOPIC, subscribers_qos, std::bind(&TruckController::reference_callback, this, std::placeholders::_1)
        );

        reset_controller_reference_msg(current_ref_, get_node());
        input_ref_.set(current_ref_);

        try
        {
            // State publisher
            controller_s_publisher_ = get_node()->create_publisher<SteeringControllerStateMsg>("~/controller_state", rclcpp::SystemDefaultsQoS());
            controller_state_publisher_ = std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
        }
        
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        this->dc_ = params_.rear_wheel_base / 2.0;
        this->Ld2_ = params_.central_wheel_base + this->dc_;
        this->Ld1_ = params_.front_wheel_base + this->Ld2_;

        RCLCPP_INFO(logger, "Configuration successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::InterfaceConfiguration TruckController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.reserve(nr_cmd_itfs_);

        for(size_t i = 0; i < params_.traction_joints_names.size(); ++i)
            command_interfaces_config.names.push_back(params_.traction_joints_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);

        for(size_t i = 0; i < params_.steering_joints_names.size(); ++i)
            command_interfaces_config.names.push_back(params_.steering_joints_names[i] + "/" + hardware_interface::HW_IF_POSITION);

        return command_interfaces_config;
    }


    controller_interface::InterfaceConfiguration TruckController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces_config.names.reserve(nr_state_itfs_);

        const auto traction_wheels_feedback = params_.position_feedback ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY;

        for(size_t i = 0; i < params_.traction_joints_names.size(); ++i)
            state_interfaces_config.names.push_back(params_.traction_joints_names[i] + "/" + traction_wheels_feedback);

        for(size_t i = 0; i < params_.steering_joints_names.size(); ++i)
            state_interfaces_config.names.push_back(params_.steering_joints_names[i] + "/" + hardware_interface::HW_IF_POSITION);

        return state_interfaces_config;
    }


    controller_interface::CallbackReturn TruckController::set_interface_numbers(size_t nr_state_itfs = 2, size_t nr_cmd_itfs = 2/*, size_t nr_ref_itfs = 2*/)
    {
        this->nr_state_itfs_ = nr_state_itfs;
        this->nr_cmd_itfs_ = nr_cmd_itfs;
        //this->nr_ref_itfs_ = nr_ref_itfs;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    void TruckController::reference_callback(const std::shared_ptr<ControllerTwistReferenceMsg> msg)
    {
        // if no timestamp provided use current time for command timestamp
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Timestamp in header is missing, using current time as command timestamp.");
          msg->header.stamp = get_node()->now();
        }

        const auto age_of_last_command = get_node()->now() - msg->header.stamp;

        if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
            input_ref_.set(*msg);

        else
        {
            RCLCPP_ERROR( get_node()->get_logger(), "Received message has timestamp %.10f older for %.10f which is more then allowed timeout (%.4f).",
            rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
            ref_timeout_.seconds());
        }
    }



    controller_interface::CallbackReturn TruckController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating TruckController");

        // Try to set default value in command. If this fails, then another command will be received soon anyways.
        reset_controller_reference_msg(current_ref_, get_node());
        input_ref_.try_set(current_ref_);

        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TruckController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (size_t i = 0; i < nr_cmd_itfs_; ++i)
        {
            if (!command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN()))
            {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set NaN value for command interface '%s' (index %zu) during deactivation.",
                    command_interfaces_[i].get_name().c_str(), i);
                return controller_interface::CallbackReturn::SUCCESS;
            }
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TruckController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!reset())
            return controller_interface::CallbackReturn::ERROR;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TruckController::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!reset())
            return controller_interface::CallbackReturn::ERROR;

        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::return_type TruckController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    {
        auto logger = get_node()->get_logger();
        
        auto current_ref_op = input_ref_.try_get();
        if (current_ref_op.has_value())
            current_ref_ = current_ref_op.value();


        const auto age_of_last_command = time - current_ref_.header.stamp;
        double linvel, angvel;

        // accept message only if there is no timeout
        if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
        {
            if (!std::isnan(current_ref_.twist.linear.x) && !std::isnan(current_ref_.twist.linear.y))
            {
                linvel = current_ref_.twist.linear.x;
                angvel = current_ref_.twist.angular.z;

                if (ref_timeout_ == rclcpp::Duration::from_seconds(0))
                {
                    current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
                    current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
                    
                    input_ref_.try_set(current_ref_);
                }
            }
        }

        else
        {
            if (!std::isnan(current_ref_.twist.linear.x) && !std::isnan(current_ref_.twist.angular.z))
            {
                linvel = std::numeric_limits<double>::quiet_NaN();
                angvel = std::numeric_limits<double>::quiet_NaN();

                current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
                current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

                input_ref_.try_set(current_ref_);
            }
        }

         // MOVE ROBOT

        // Limit velocities and accelerations:
        // TODO(destogl): add limiter for the velocities

        if(!std::isnan(linvel) && !std::isnan(angvel))
        {
            // compute the commands for the steering and traction joints based on the current reference
            // and the kinematic model of the truck
            double turningRadius;
            std::vector<double> steering_commands(params_.steering_joints_names.size(), 0.0);
            std::vector<double> traction_commands(params_.traction_joints_names.size(), 0.0);
        
            //if angvel is too small, it is assumed that truck should not be turning (i.e., should be going straight)
            bool isTurning = true;

            if (std::abs(angvel) < 1e-5)
                isTurning = false;
        
            if (isTurning)
            {
                //turning radius calculation
                turningRadius = linvel / angvel;

                 //distance of application point from ICR along x-axis (i.e., axis orthogonal to the truck heading axis)
                double turningRadius_x;

                /*compute turningRadius_x by applying Pythagoras theorem.
                If the calculation is impossible due to turningRadius being smaller than the application point
                offset, then set a very small, arbitrary turningRadius_x*/
                if (std::abs(turningRadius) >= params_.app_point_offset)
                    turningRadius_x = (turningRadius >= 0 ? 1.0 : -1.0) * std::sqrt(std::pow(turningRadius, 2) - std::pow(params_.app_point_offset, 2));
                else
                    turningRadius_x = (turningRadius >= 0 ? 1.0 : -1.0) * 1e-1;
                
                /*check that turningRadius_x is not so small that the maximum steering angles would be exceeded.
                If it is, then redefine turningRadius_x and angvel to match the steering limit*/
                double maxDelta = std::atan(this->Ld1_ / (std::abs(turningRadius_x) - params_.front_wheel_separation/2.0));
                if (maxDelta >= params_.steering_limit)
                {
                    double sgn_turningRadius = (turningRadius_x >= 0 ? 1.0 : -1.0);
                    turningRadius_x = sgn_turningRadius * (this->Ld1_ / std::tan(params_.steering_limit) + params_.front_wheel_separation/2.0);
                    angvel = (angvel >= 0 ? 1.0 : -1.0) * std::abs(linvel / std::sqrt(std::pow(turningRadius_x, 2) + std::pow(params_.app_point_offset, 2)));
                }

                auto calc_wheel_steer = [&](double L, double R) {return (R >= 0 ? 1.0 : -1.0) * std::atan2(L, std::abs(R));};

                //Compute steering angles
                steering_commands =
                {
                  /*std::atan(this->Ld1_/ (turningRadius_x - params_.front_wheel_separation/2.0)), //delta1_L
                    std::atan(this->Ld1_/ (turningRadius_x + params_.front_wheel_separation/2.0)), //delta1_R
                    std::atan(this->Ld2_/ (turningRadius_x - params_.front_wheel_separation/2.0)), //delta2_L
                    std::atan(this->Ld2_/ (turningRadius_x + params_.front_wheel_separation/2.0)) //delta2_R
                    */
                    calc_wheel_steer(this->Ld1_, turningRadius_x - params_.front_wheel_separation/2.0), //delta1_L
                    calc_wheel_steer(this->Ld1_, turningRadius_x + params_.front_wheel_separation/2.0), //delta1_R
                    calc_wheel_steer(this->Ld2_, turningRadius_x - params_.front_wheel_separation/2.0), //delta2_L
                    calc_wheel_steer(this->Ld2_, turningRadius_x + params_.front_wheel_separation/2.0) //delta2_R
                };

                //Compute wheel speeds
                auto calc_wheel_speed = [&](double L, double x_off) {
                    double dist_to_icr = std::sqrt(std::pow(turningRadius_x - x_off, 2) + std::pow(L, 2));
                    double wheel_v = std::abs(angvel) * dist_to_icr;
                    return (linvel >= 0 ? 1.0 : -1.0) * (wheel_v / params_.wheel_radius);
                };

                //Compute traction speeds
                if (!params_.full_traction)
                    traction_commands = {
                                            calc_wheel_speed(this->dc_, params_.rear_wheel_separation/2.0), //omega3_L
                                            calc_wheel_speed(this->dc_, -params_.rear_wheel_separation/2.0), //omega3_R
                                            calc_wheel_speed(this->dc_, params_.rear_wheel_separation/2.0), //omega4_L
                                            calc_wheel_speed(this->dc_, -params_.rear_wheel_separation/2.0) //omega4_R
                                        };
                
                else
                    traction_commands = {
                                            calc_wheel_speed(this->Ld1_, params_.front_wheel_separation/2.0), //omega1_L
                                            calc_wheel_speed(this->Ld1_, -params_.front_wheel_separation/2.0), //omega1_R
                                            calc_wheel_speed(this->Ld2_, params_.front_wheel_separation/2.0), //omega2_L
                                            calc_wheel_speed(this->Ld2_, -params_.front_wheel_separation/2.0), //omega2_R
                                            calc_wheel_speed(this->dc_, params_.rear_wheel_separation/2.0), //omega3_L
                                            calc_wheel_speed(this->dc_, -params_.rear_wheel_separation/2.0), //omega3_R
                                            calc_wheel_speed(this->dc_, params_.rear_wheel_separation/2.0), //omega4_L
                                            calc_wheel_speed(this->dc_, -params_.rear_wheel_separation/2.0) //omega4_R
                                        };

            }// enf if turning

            else
            {
                //Compute steering angles
                steering_commands = {0.0, 0.0, 0.0, 0.0};

                //Compute traction speeds
                if (!params_.full_traction)
                    traction_commands = { linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius};
                
                else
                    traction_commands =
                    {
                        linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius,
                        linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius, linvel/ params_.wheel_radius
                    };
            }

            for (size_t i = 0; i < params_.traction_joints_names.size(); i++)
            {
                const auto & value = traction_commands[i];

                if (!command_interfaces_[i].set_value(value))
                {
                    RCLCPP_WARN(logger, "Unable to set traction command at index %zu: value = %f", i, value);
                    return controller_interface::return_type::OK;
                }
            }

            for (size_t i = 0; i < params_.steering_joints_names.size(); i++)
            {
                const auto & value = steering_commands[i];
                
                if (!command_interfaces_[i + params_.traction_joints_names.size()].set_value(value))
                {
                    RCLCPP_WARN(logger, "Unable to set steering command at index %zu: value = %f", i, value);
                    return controller_interface::return_type::OK;
                }
            }
        }// first if

        else
        {
            for (size_t i = 0; i < params_.traction_joints_names.size(); i++)
                if (!command_interfaces_[i].set_value(0.0))
                {
                    RCLCPP_WARN(logger, "Unable to set command interface to value 0.0");
                    return controller_interface::return_type::OK;
                }
            
            // Hold steering joints straight
            for (size_t i = 0; i < params_.steering_joints_names.size(); i++)
            {
                const size_t steering_index = i + params_.traction_joints_names.size();
                
                if (!command_interfaces_[steering_index].set_value(0.0))
                {
                    RCLCPP_WARN(logger, "Unable to set steering command interface to value 0.0");
                    return controller_interface::return_type::OK;
                }
            }
        }


        if (controller_state_publisher_)
        {
            controller_state_msg_.header.stamp = time;
            controller_state_msg_.traction_wheels_position.clear();
            controller_state_msg_.traction_wheels_velocity.clear();
            controller_state_msg_.linear_velocity_command.clear();
            controller_state_msg_.steer_positions.clear();
            controller_state_msg_.steering_angle_command.clear();

            auto number_of_traction_wheels = params_.traction_joints_names.size();
            auto number_of_steering_wheels = params_.steering_joints_names.size();

            for (size_t i = 0; i < number_of_traction_wheels; ++i)
            {
                if (params_.position_feedback)
                {
                    auto position_state_interface_op = state_interfaces_[i].get_optional();
                    if (!position_state_interface_op.has_value())
                        RCLCPP_DEBUG(logger, "Unable to retrieve position feedback data for traction wheel %zu", i);

                    else
                        controller_state_msg_.traction_wheels_position.push_back(position_state_interface_op.value());
                }

                else
                {
                    auto velocity_state_interface_op = state_interfaces_[i].get_optional();
                    if (!velocity_state_interface_op.has_value())
                        RCLCPP_DEBUG(logger, "Unable to retrieve velocity feedback data for traction wheel %zu", i);

                    else
                        controller_state_msg_.traction_wheels_velocity.push_back(velocity_state_interface_op.value());
                }

                auto velocity_command_interface_op = command_interfaces_[i].get_optional();
                if (!velocity_command_interface_op.has_value())
                    RCLCPP_DEBUG(logger, "Unable to retrieve velocity command for traction wheel %zu", i);

                else
                    controller_state_msg_.linear_velocity_command.push_back(velocity_command_interface_op.value());
            }

            for (size_t i = 0; i < number_of_steering_wheels; ++i)
            {
                const auto state_interface_value_op = state_interfaces_[number_of_traction_wheels + i].get_optional();
                const auto command_interface_value_op = command_interfaces_[number_of_traction_wheels + i].get_optional();

                if (!state_interface_value_op.has_value() || !command_interface_value_op.has_value())
                    RCLCPP_DEBUG(logger, "Unable to retrieve %s for steering wheel %zu",
                    !state_interface_value_op.has_value() ? "state interface value" : "command interface value", i);
                else
                {
                    controller_state_msg_.steer_positions.push_back(state_interface_value_op.value());
                    controller_state_msg_.steering_angle_command.push_back(command_interface_value_op.value());
                }
            }

            controller_state_publisher_->try_publish(controller_state_msg_);
        }

        
        return controller_interface::return_type::OK;
    }


    bool TruckController::reset()
    {
        reset_controller_reference_msg(current_ref_, get_node());
        input_ref_.set(current_ref_);

        ref_subscriber_twist_.reset();

        return true;
    }

}// end namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(truck_controller::TruckController, controller_interface::ControllerInterface)