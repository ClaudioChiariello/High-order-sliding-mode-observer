#ifndef TRUCK_CONTROLLER__TRUCK_CONTROLLER_HPP_
#define TRUCK_CONTROLLER__TRUCK_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/steering_controller_status.hpp"

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include <memory>
#include <vector>
#include <string>

#include "truck_controller_pkg/truck_controller_parameters.hpp"

namespace truck_controller{
class TruckController : public controller_interface::ControllerInterface
{
    
    public:
        TruckController(); 

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
        using SteeringControllerStateMsg = control_msgs::msg::SteeringControllerStatus;

        bool reset();


    protected:

        // save the last reference in case of unable to get value from box
        ControllerTwistReferenceMsg current_ref_;
        rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms
        rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_twist_ = nullptr;
        // the RT Box containing the command message
        realtime_tools::RealtimeThreadSafeBox<ControllerTwistReferenceMsg> input_ref_;

        // Parameters from ROS for diff_drive_controller
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        using ControllerStatePublisher = realtime_tools::RealtimePublisher<SteeringControllerStateMsg>;
        rclcpp::Publisher<SteeringControllerStateMsg>::SharedPtr controller_s_publisher_;
        std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;
        SteeringControllerStateMsg controller_state_msg_;

        // name constants for state interfaces
        size_t nr_state_itfs_;
        // name constants for command interfaces
        size_t nr_cmd_itfs_;
        // name constants for reference interfaces
        //size_t nr_ref_itfs_;

        controller_interface::CallbackReturn set_interface_numbers(size_t nr_state_itfs, size_t nr_cmd_itfs/*, size_t nr_ref_itfs*/);

        double Ld1_, Ld2_, dc_;
        std::vector<std::string> traction_joints_state_names_;
        std::vector<std::string> steering_joints_state_names_;

    private:
        // callback for topic interface
        void reference_callback(const std::shared_ptr<ControllerTwistReferenceMsg> msg);
        
};
}

#endif //TRUCK_CONTROLLER__TRUCK_CONTROLLER_HPP_