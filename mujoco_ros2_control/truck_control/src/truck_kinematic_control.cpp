#include "truck_control/truck_controller.hpp"
#include <cmath>

namespace truck_control{
class TruckKinematicController : public TruckController
{
public:
    TruckKinematicController(const std::string &node_name)
    : TruckController(node_name)
    {}

    void control_wheels(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        //linear and angular speeds are extracted from Twist message
        double v = msg->linear.x;
        double omega = msg->angular.z;
        
        //Desired steering angles
        std_msgs::msg::Float64MultiArray steer_msg;
        //Desired traction speeds
        std_msgs::msg::Float64MultiArray traction_msg;

        /*the turning radius, defined as the distance of application point from ICR.
        ICR is assumed to be always along intermediate axis between rear wheel axes*/
        double turningRadius;
        
        //if omega is too small, it is assumed that truck should not be turning (i.e., should be going straight)
        bool isTurning = true;
        if (std::abs(omega) < 1e-5)  isTurning = false;
        
        if (isTurning)
        {
            //turning radius calculation
            turningRadius = v / omega;
            
            //distance of application point from ICR along x-axis (i.e., axis orthogonal to the truck heading axis)
            double turningRadius_x;

            /*compute turningRadius_x by applying Pythagoras theorem.
            If the calculation is impossible due to turningRadius being smaller than the application point
            offset, then set a very small, arbitrary turningRadius_x*/
            if (std::abs(turningRadius) >= app_point_offset_)
                turningRadius_x = (turningRadius >= 0 ? 1.0 : -1.0) * std::sqrt(std::pow(turningRadius, 2) - std::pow(app_point_offset_, 2));
            else
                turningRadius_x = (turningRadius >= 0 ? 1.0 : -1.0) * 1e-1;
            
            /*check that turningRadius_x is not so small that the maximum steering angles would be exceeded.
            If it is, then redefine turningRadius_x and omega to match the steering limit*/
            double maxDelta = std::atan2(L1_, std::abs(turningRadius_x) - front_wheel_separation_/2.0);
            if (maxDelta >= steering_limit_)
            {
                double sgn_turningRadius = (turningRadius_x >= 0 ? 1.0 : -1.0);
                turningRadius_x = sgn_turningRadius * (L1_ / std::tan(steering_limit_) + front_wheel_separation_/2.0);
                omega = (omega >= 0 ? 1.0 : -1.0) * std::abs(v / std::sqrt(std::pow(turningRadius_x, 2) + std::pow(app_point_offset_, 2)));
            }

            //Compute steering angles
            steer_msg.data =
            {
                std::atan(L1_/ (turningRadius_x - front_wheel_separation_/2.0)), //delta1_L
                std::atan(L1_/ (turningRadius_x + front_wheel_separation_/2.0)), //delta1_R
                std::atan(L2_/ (turningRadius_x - front_wheel_separation_/2.0)), //delta2_L
                std::atan(L2_/ (turningRadius_x + front_wheel_separation_/2.0)) //delta2_R
            };

            //Compute wheel speeds
            auto calc_wheel_speed = [&](double L, double x_off) {
                double dist_to_icr = std::sqrt(std::pow(turningRadius_x - x_off, 2) + std::pow(L, 2));
                double wheel_v = std::abs(omega) * dist_to_icr;
                return (v >= 0 ? 1.0 : -1.0) * (wheel_v / wheel_radius_);
            };

            //Compute traction speeds
            if (driven_wheels_ == 4)
            {
                traction_msg.data =
                {
                    calc_wheel_speed(L3_, rear_wheel_separation_/2.0), //omega3_L
                    calc_wheel_speed(L3_, -rear_wheel_separation_/2.0), //omega3_R
                    calc_wheel_speed(L4_, rear_wheel_separation_/2.0), //omega4_L
                    calc_wheel_speed(L4_, -rear_wheel_separation_/2.0) //omega4_R
                };
            }
            else
            {
                traction_msg.data =
                {
                    calc_wheel_speed(L1_, front_wheel_separation_/2.0), //omega1_L
                    calc_wheel_speed(L1_, -front_wheel_separation_/2.0), //omega1_R
                    calc_wheel_speed(L2_, front_wheel_separation_/2.0), //omega2_L
                    calc_wheel_speed(L2_, -front_wheel_separation_/2.0), //omega2_R
                    calc_wheel_speed(L3_, rear_wheel_separation_/2.0), //omega3_L
                    calc_wheel_speed(L3_, -rear_wheel_separation_/2.0), //omega3_R
                    calc_wheel_speed(L4_, rear_wheel_separation_/2.0), //omega4_L
                    calc_wheel_speed(L4_, -rear_wheel_separation_/2.0) //omega4_R
                };
            }
          
        }
        else
        {
            //Compute steering angles
            steer_msg.data = {0.0, 0.0, 0.0, 0.0};

            //Compute traction speeds
            if (driven_wheels_ == 4)
            {
                traction_msg.data =
                {
                    v / wheel_radius_, v / wheel_radius_, v / wheel_radius_, v / wheel_radius_
                };
            }
            else
            {
                traction_msg.data =
                {
                    v / wheel_radius_, v / wheel_radius_, v / wheel_radius_, v / wheel_radius_,
                    v / wheel_radius_, v / wheel_radius_, v / wheel_radius_, v / wheel_radius_
                };
            }
        }

        //Finally, publish the messages
        steer_publisher_->publish(steer_msg);
        traction_publisher_->publish(traction_msg);
    }
};
}//end of namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<truck_control::TruckKinematicController>("truck_kinematic_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}