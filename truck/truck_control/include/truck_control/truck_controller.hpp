#ifndef TRUCK_CONTROL_HPP
#define TRUCK_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace truck_control{
class TruckController : public rclcpp::Node
{
public:

    virtual ~TruckController() = default;
    
    TruckController(const std::string &node_name)
    :Node(node_name)
    {
        /*Extraction of truck geometric parameters as declared in launch file.
        If a tag is not present, default values are used*/
        this->declare_parameter<int>("driven_wheels", 8);
        this->declare_parameter<double>("rear_wheel_separation", 2.088);
        this->declare_parameter<double>("front_wheel_separation", 2.131);
        this->declare_parameter<double>("wheel_radius", 0.5645);
        this->declare_parameter<double>("front_wheel_base", 1.9);
        this->declare_parameter<double>("central_wheel_base", 3.75);
        this->declare_parameter<double>("rear_wheel_base", 1.382);
        this->declare_parameter<double>("steering_limit", 0.61);
        this->declare_parameter<double>("app_point_offset", 0.0);

        //Assignment of geometric parameters to class data members
        this->get_parameter("driven_wheels", driven_wheels_);
        this->get_parameter("rear_wheel_separation", rear_wheel_separation_);
        this->get_parameter("front_wheel_separation", front_wheel_separation_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("front_wheel_base", front_wheel_base_);
        this->get_parameter("central_wheel_base", central_wheel_base_);
        this->get_parameter("rear_wheel_base", rear_wheel_base_);
        this->get_parameter("steering_limit", steering_limit_);
        this->get_parameter("app_point_offset", app_point_offset_);

        //Computation of derived geometric parameters
        dc_ = rear_wheel_base_ / 2.0;
        L4_ = -dc_;
        L3_ = dc_;
        L2_ = L3_ + central_wheel_base_;
        L1_ = L2_ + front_wheel_base_;

        /*Subscription to the cmd_vel topic to receive desired linear and rotational speeds
        for the chosen application point of the truck*/
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){
                this->control_wheels(msg);
            });
        
        /*Publishers to topics to control individual wheels, via the control_wheels function.
        Controllers for both traction and steering joints are defined*/
        steer_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("steering_controller/commands", 10);
        traction_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("traction_controller/commands", 10);
    }

    //virtual function that implements control for each wheel, to be overridden by each specific inherited controller
    virtual void control_wheels(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
    
protected:
    //Basic geometric parameters
    int driven_wheels_;
    double rear_wheel_separation_;
    double front_wheel_separation_;
    double wheel_radius_;
    double front_wheel_base_;
    double central_wheel_base_;
    double rear_wheel_base_;
    double steering_limit_;
    double app_point_offset_;

    //Derived geometric parameters
    double dc_;
    double L1_;
    double L2_;
    double L3_;
    double L4_;

    //traction and steering publishers to control individual wheels
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traction_publisher_;

private:
    //cmd_vel subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};
} //end of namespace
#endif