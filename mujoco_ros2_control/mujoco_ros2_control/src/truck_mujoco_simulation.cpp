
#include "mujoco_ros2_control/truck_mujoco_simulation.hpp"
#include <iostream>
#include <iomanip>
#include <nav_msgs/msg/odometry.hpp>

namespace mujoco_ros2_control_truck{


bool GetSimData::initialize(
    rclcpp::Node::SharedPtr node, 
    const std::string& model_path, 
    const std::string& mujoco_model_topic,
    double sim_speed_factor, 
    bool headless)
{
    /*Call the parent initialize method
        - Here they initialize the model object
        - Initialize the cam
    */
    if (!MujocoSimulation::initialize(node, model_path, mujoco_model_topic, sim_speed_factor, headless))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize base MujocoSimulation");
        return false;
    }

    custom_node_ = node;

    // 2. Setup ROS 2 publishers using the node handle
    wrench_pub_ = custom_node_->create_publisher<mujoco_ros2_control_msgs::msg::BodyWrench>("/wrenches", 10);
    wrenches_from_actuators_pub_ = custom_node_->create_publisher<mujoco_ros2_control_msgs::msg::BodyWrench>("/wrenches_from_actuators", 10);
    imu_pub_ = custom_node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    contact_state_pub_ = custom_node_->create_publisher<mujoco_ros2_control_msgs::msg::ContactState>("/contact_states", 10);
    odom_pub_ = custom_node_->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(custom_node_);

    return true;
}

 
void GetSimData::print_all_joint_torques(const mjModel* m, mjData* d) {
    // Method body
}

void GetSimData::publish_wrenches(){

    std::lock_guard<std::recursive_mutex> lock(this->getMutex());

    // model() and data() sono funzioni pubbliche, e posso accedere agli oggetti

    mjModel* model = this->model();
    mjData* data = this->data();
    
    int base_link_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    int base_footprint_id = mj_name2id(model, mjOBJ_BODY, "base_footprint");
    
    publish_odometry(base_footprint_id, model, data);

    getGroundContactWrench(base_link_id, model, data);

    int imu_link_id = mj_name2id(model, mjOBJ_BODY, "imu_link");
    publish_imu_data_kinematics(imu_link_id, model, data);

    // const mjtNum* wrench = data->cfrc_int + 6 * base_link_id;

    // mujoco_ros2_control_msgs::msg::BodyWrench msg;
    
    // msg.header.stamp = node_->now();
    // msg.header.frame_id = "base_link";

    // msg.body_name = "base_link";

    // msg.wrench.force.x = wrench[3];
    // msg.wrench.force.y = wrench[4];
    // msg.wrench.force.z = wrench[5];

    // msg.wrench.torque.x = wrench[0];
    // msg.wrench.torque.y = wrench[1];
    // msg.wrench.torque.z = wrench[2];

    // wrench_pub_->publish(msg);
}


void GetSimData::publish_imu_data_kinematics(int body_id, const mjModel* model,  mjData* data){

    std::lock_guard<std::recursive_mutex> lock(this->getMutex());

    const char* body_name_char = mj_id2name(model, mjOBJ_BODY, body_id);
    if (body_id < 0) return;

    // compute forces and acceleration

    mj_rnePostConstraint(model, data);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = body_name_char;

    // 1. Convert Rotation Matrix (xmat) to Quaternion
    mjtNum quat[4]; // [w, x, y, z]
    mju_mat2Quat(quat, data->xmat + 9 * body_id);

    msg.orientation.w = quat[0];
    msg.orientation.x = quat[1];
    msg.orientation.y = quat[2];
    msg.orientation.z = quat[3];

    
    // 2. Angular velocity from 6D spatial velocity vector (cvel: [ang_x, ang_y, ang_z, lin_x, lin_y, lin_z])
    const mjtNum* cvel = data->cvel + 6 * body_id;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(cvel[i]) || std::abs(cvel[i]) > 10.0) {
            std::cout << "[WARNING] Angular velocity spike detected on body '" 
                    << body_name_char << "': [" 
                    << cvel[0] << ", " << cvel[1] << ", " << cvel[2] << "]" << std::endl;
            break;
        }
    }

    msg.angular_velocity.x = static_cast<double>(cvel[0]);
    msg.angular_velocity.y = static_cast<double>(cvel[1]);
    msg.angular_velocity.z = static_cast<double>(cvel[2]);
 

    // 3. Linear acceleration from 6D spatial acceleration vector (cacc: [ang_acc..., lin_acc...])
    //const mjtNum* acc = data->cacc + 6 * body_id;
    int id = mj_name2id(model, mjOBJ_SENSOR, "imu_accel");
    int adr = model->sensor_adr[id];
    const mjtNum* acc = data->sensordata + adr;
    msg.linear_acceleration.x = acc[0];
    msg.linear_acceleration.y = acc[1];
    msg.linear_acceleration.z = acc[2]; // Add gravity vector if simulating physical accelerometer

    imu_pub_->publish(msg);
}



void GetSimData::getGroundContactWrench(int body_id, const mjModel* model, mjData* data ){

   
    mujoco_ros2_control_msgs::msg::ContactState msg;

    msg.header.stamp = node_->now(); 
    const char* body_name_char = mj_id2name(model, mjOBJ_BODY, body_id);
    msg.header.frame_id = body_name_char;
 

    // 2. Get the target body's Center of Mass (COM) position in World Frame
    const mjtNum* body_CoM = &data->xipos[3 * body_id];
    // 3x3 Rotation matrix of base_link in World Frame
    const mjtNum* R_world_baselink = &data->xmat[9 * body_id];

    // Pre-allocate buffer on stack (zero overhead, no memory leaks)
    double Fy_paceika[4]; 
    // Call function and pass pointer
    double Fy_tot = Paceika_model(model, data, Fy_paceika);

    double Fn_r = 0;
    double Fn_l = 0;
    for (int i = 0; i < data->ncon; ++i) {
        
        // Single contact sub-message
        mujoco_ros2_control_msgs::msg::ContactPair contact;

        // Retrieve the contact position and the body in contact frame
        const mjContact& con = data->contact[i];
        //con.geom1 and con.geom2 are the 2 contact geometries

        // 6D force in contact frame: [normal, friction_x, friction_y, torque_r, torque_p, torque_y]
        int body1 = model->geom_bodyid[con.geom1]; //world
        int body2 = model->geom_bodyid[con.geom2];

        const char* name1 = mj_id2name(model, mjOBJ_BODY, body1);
        const char* name2 = mj_id2name(model, mjOBJ_BODY, body2);

        contact.body1_name = name1 ? name1 : "unnamed_body_" + std::to_string(body1);
        contact.body2_name = name2 ? name2 : "unnamed_body_" + std::to_string(body2);
        
        contact.position.x = con.pos[0];
        contact.position.y = con.pos[1];
        contact.position.z = con.pos[2];


        // 6D force in contact frame: [normal, friction_x, friction_y, torque_r, torque_p, torque_y]
        mjtNum c_force[6];
        mj_contactForce(model, data, i, c_force);

        // Sign flip depending on whether body_id is body1 or body2 (Newton's 3rd Law)
        // con.frame is the rotation matrix. The normal is in [0-2], points from geom[0] to geom[1]
        double sign = (body1 == body_id) ? 1.0 : -1.0;

        // --- LINEAR FORCE ---
        // 1. Transform Contact Frame -> World Frame con.Frame è la rotation matrix R^W_c
        mjtNum f_world[3];
        mju_mulMatTVec(f_world, con.frame, c_force, 3, 3);

        // 2. Transform World Frame -> base_link Frame
        mjtNum f_base_link[3];
        mju_mulMatTVec(f_base_link, R_world_baselink, f_world, 3, 3);
    
        // Aplpy action-reaction sign flip
        contact.force.x = f_base_link[0];
        contact.force.y = Fy_tot;
        contact.force.z = f_base_link[2]; //is positive because the normal points from geom 0 to to geom 1 (upwards)
        
        if (contact.body2_name.find("right") != std::string::npos) {
            Fn_r += f_base_link[2];
        } 
        else if (contact.body2_name.find("left") != std::string::npos) {
            Fn_l += f_base_link[2];
        }
        
        
        // --- TORQUE & MOMENT ---
        // 1. Torsional Contact Torque: Contact Frame -> World Frame -> base_link Frame
        mjtNum t_contact_world[3];
        mju_mulMatTVec(t_contact_world, con.frame, &c_force[3], 3, 3);
        //The third argument needs the address of an element, here I am giving the address of the third element of c_force, that is the torsional contact forces

        mjtNum t_contact_base_link[3];
        mju_mulMatTVec(t_contact_base_link, R_world_baselink, t_contact_world, 3, 3);

        // Calculate moment arm: r = contact_position - body_CoM in the WORLD FRAME
        mjtNum r_world[3] = {
        con.pos[0] - body_CoM[0],
        con.pos[1] - body_CoM[1],
        con.pos[2] - body_CoM[2]
        };

        // 3. Induced moment in World Frame: tau_moment = r_world x F_world
        mjtNum t_moment_world[3];
        mju_cross(t_moment_world, r_world, f_world);

        // 4. Transform induced moment: World Frame -> base_link Frame
        mjtNum t_moment_base_link[3];
        mju_mulMatTVec(t_moment_base_link, R_world_baselink, t_moment_world, 3, 3);

        // 5. Total torque (Torsional + Induced Moment) with sign flip
        // Uncomment + t_moment_base_link if moment about COM is required
        contact.torque.x = (t_contact_base_link[0] /* + t_moment_base_link[0] */) * sign;
        contact.torque.y = (t_contact_base_link[1] /* + t_moment_base_link[1] */) * sign;
        contact.torque.z = (t_contact_base_link[2] /* + t_moment_base_link[2] */) * sign;

        msg.contacts.push_back(contact); 
    
    }
       
    double LTR = (Fn_r - Fn_l) / (Fn_r + Fn_l );
    msg.ltr = LTR;

    contact_state_pub_->publish(msg);

}



void GetSimData::publish_odometry(int body_id, const mjModel* model, mjData* data )
{

    rclcpp::Time current_time = node_->now();
    const char* body_name_char = mj_id2name(model, mjOBJ_BODY, body_id);

    // 1. Retrieve 3D position (x, y, z) from mjData. 3* bodyId is because every position is stored consecutively in a vector 3 x num_link;
    /* pos 0 you get the pos_x of link 0 --- pos 1 you get the pos_y of the link 0 --- pos 2 you get the pos_z of the link 0
    pos 3 you get the pos_x of link 1 --- pos 4 you get the pos_y of the link 1 --- pos 5 you get the pos_z of the link 1
    ....
    pos 3*body_Id you get the pos_x of link "body_id" --- pos  3*body_Id + 1 you get the pos_y of the link  body_Id --- pos  3*body_Id + 2 you get the pos_z of the link bodyID */
    double pos_x = data->xpos[3 * body_id + 0];
    double pos_y = data->xpos[3 * body_id + 1];
    double pos_z = data->xpos[3 * body_id + 2];

    // 2. Retrieve orientation quaternion (w, x, y, z) from mjData
    double quat_w = data->xquat[4 * body_id + 0];
    double quat_x = data->xquat[4 * body_id + 1];
    double quat_y = data->xquat[4 * body_id + 2];
    double quat_z = data->xquat[4 * body_id + 3];

    // retrieve the velocity in the world frame
    mjtNum vel[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, body_id, vel, 0);
    
     // Pointers to world angular (0..2) and linear (3..5) velocities
    const mjtNum* world_ang_vel = vel;     
    const mjtNum* world_lin_vel = vel + 3; 


    mjtNum R[9];
    mju_copy(R, &data->xmat[body_id * 9], 9);
    // Multiply the 1st column by -1 (indices 0, 3, and 6)-
    // Faccio questo perchè sospetto che il base_link e il world_link abbiano l'asse x in direzioni opposte e quando il camion va 5m/s mi trovo che dice che sta a -5m/s
  
    R[0] *= 1.0;
    R[3] *= 1.0;
    R[6] *= -1.0;

    // Siccome &data->xmat[body_id * 9] è un indirizzo di memoria, andrebbe assegnato a un puntatore a mjtNum
    // mjtNum* R = &data->xmat[body_id * 9];
 
    // 2. Rotate World -> Body frame: v_body = R^T * v_world
    mjtNum body_linear_vel[3];
    mjtNum body_angular_vel[3];

    // mju_mulMatTVec multiplies Transpose(R) * world_lin_vel
    mju_mulMatTVec(body_linear_vel, R, world_lin_vel, 3, 3);
    mju_mulMatTVec(body_angular_vel, R, world_ang_vel, 3, 3);


    // -------------------------------------------------------------
    // Populate Odometry Message
    // -------------------------------------------------------------
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = body_name_char;

    // Position
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.position.z = pos_z;

    // Orientation
    odom_msg.pose.pose.orientation.w = quat_w;
    odom_msg.pose.pose.orientation.x = quat_x;
    odom_msg.pose.pose.orientation.y = quat_y;
    odom_msg.pose.pose.orientation.z = quat_z;

    // Twist (Velocities)
    odom_msg.twist.twist.linear.x = body_linear_vel[0];
    odom_msg.twist.twist.linear.y = body_linear_vel[1];
    odom_msg.twist.twist.linear.z = body_linear_vel[2];

    odom_msg.twist.twist.angular.x = body_angular_vel[0];
    odom_msg.twist.twist.angular.y = body_angular_vel[1];
    odom_msg.twist.twist.angular.z = body_angular_vel[2];

    // Publish Odometry
    odom_pub_->publish(odom_msg);

    // -------------------------------------------------------------
    // Broadcast TF (odom -> base_footprint)
    // -------------------------------------------------------------
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = body_name_char;

    tf_msg.transform.translation.x = pos_x;
    tf_msg.transform.translation.y = pos_y;
    tf_msg.transform.translation.z = pos_z;

    tf_msg.transform.rotation.w = quat_w;
    tf_msg.transform.rotation.x = quat_x;
    tf_msg.transform.rotation.y = quat_y;
    tf_msg.transform.rotation.z = quat_z;

    // double vx_times_wz = body_linear_vel[0] * body_angular_vel[2];

    // std::cout << " | vx*wz = " << vx_times_wz
    //         << std::endl;

    tf_broadcaster_->sendTransform(tf_msg);
    
    body_lin_vel_ = {{body_linear_vel[0], body_linear_vel[1], body_linear_vel[2]}};  
    //double yaw   = std::atan2(R[3], R[0]);
    heading_vel_ = body_angular_vel[2];

}


double GetSimData::Paceika_model(const mjModel* model, const mjData* data, double * Fy)
{

    //std::array<double, 4> L = custom_node_->get_parameter("L").as_double();
    std::array<double, 4> L = {{3.65, 1.75, -2.0, -3.39}};
    const std::array<std::string, 2> left_steer_joints = {{"left_steer_joint_a1", "left_steer_joint_a2"}};

    double delta[2] = {0.0, 0.0};
    double C_alpha[4] = {350e2, 350e2, 350e2, 350e2};

    double vx = (std::abs(body_lin_vel_[0]) < 1e-3) ? 1e-3 : body_lin_vel_[0];

    for (int i = 0; i < 4; i++){
        if (i <2){
            int joint_id = mj_name2id(model, mjOBJ_JOINT, left_steer_joints[i].c_str());

            // 2. Get the memory addresses for position and velocity in qpos and qvel
            int qpos_addr = model->jnt_qposadr[joint_id];
            *(delta + i) = data->qpos[qpos_addr];


            *(Fy + i) = -C_alpha[i] *  ( delta[i] - (body_lin_vel_[1] + L[i] * heading_vel_) / vx ) ;
        }
        else
            *(Fy + i) = -C_alpha[i] *  ( - (body_lin_vel_[1] + L[i] * heading_vel_) / vx ) ;
    }

    return Fy[0] * std::cos(delta[0]) +  Fy[1] * std::cos(delta[1])  + Fy[2] +  Fy[3];

}
}