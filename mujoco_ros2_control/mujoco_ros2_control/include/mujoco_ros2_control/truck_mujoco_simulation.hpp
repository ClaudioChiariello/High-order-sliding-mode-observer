
#include "mujoco_ros2_control/mujoco_simulation.hpp" 

#include "mujoco_ros2_control_msgs/msg/contact_pair.hpp"
#include "mujoco_ros2_control_msgs/msg/contact_state.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include <sensor_msgs/msg/imu.hpp>



#include <mujoco/mujoco.h>


namespace mujoco_ros2_control_truck{


    class GetSimData;

    class GetSimData: public mujoco_ros2_control::MujocoSimulation
    {
    public:
        GetSimData() = default; 
        ~GetSimData() = default;

        bool initialize(
            rclcpp::Node::SharedPtr node, 
            const std::string& model_path, 
            const std::string& mujoco_model_topic,
            double sim_speed_factor, 
            bool headless);

        void publish_wrenches();

        // bool getBodyVelocity(
        //     const std::string& body_name,
        //     mjtNum* lin,
        //     mjtNum* ang,
        //     bool world_frame = true);

        //void publish_actuator_wrenches();
        
        void start_physics_thread();
        
        void print_all_joint_torques(const mjModel* m, mjData* d);

        void publish_odometry(int body_id, const mjModel* model, mjData* data);

        void getGroundContactWrench(int body_id, const mjModel* model, mjData* data);

        void publish_imu_data_kinematics(int body_id, const mjModel* model, mjData* data);

        double Paceika_model(const mjModel* model, const mjData* data, double * Fy);

    // protected:
    //     // Reimplement physics_loop to override the base class behavior
    //     void physics_loop() override;


    private:

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Publisher<mujoco_ros2_control_msgs::msg::BodyWrench>::SharedPtr wrench_pub_;
        rclcpp::Publisher<mujoco_ros2_control_msgs::msg::BodyWrench>::SharedPtr wrenches_from_actuators_pub_;

        rclcpp::Publisher<mujoco_ros2_control_msgs::msg::ContactState>::SharedPtr contact_state_pub_;

        rclcpp::Node::SharedPtr custom_node_;

        int count_ = 0;

        std::array<double, 3> body_lin_vel_ = {{0.0, 0.0, 0.0}};
        
        double heading_vel_ = 0;

    };
}