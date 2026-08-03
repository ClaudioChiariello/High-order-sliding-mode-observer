
#include "mujoco_ros2_control/truck_mujoco_simulation.hpp"
#include <iostream>
#include <iomanip>
#include <nav_msgs/msg/odometry.hpp>
#include <mujoco_ros2_control_msgs/msg/contact_forces.hpp>




using namespace std::chrono_literals;

// constants
const double kSyncMisalign = 0.1;        // maximum misalignment before re-sync (simulation seconds)
const double kSimRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;           // load error string length

using Seconds = std::chrono::duration<double>;



namespace mujoco_ros2_control_truck{


    bool GetSimData::initialize(
        rclcpp::Node::SharedPtr node, 
        const std::string& model_path, 
        const std::string& mujoco_model_topic,
        double sim_speed_factor, 
        bool headless)
        {

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
        contact_forces_pub_ = custom_node_->create_publisher<mujoco_ros2_control_msgs::msg::ContactForces>("/contact_forces", 10);
        odom_pub_ = custom_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(custom_node_);

        return true;
    }



//     // simulate in background thread (while rendering in main thread)
// void GetSimData::physics_loop()
// {
//   // cpu-sim synchronization point
//   std::chrono::time_point<mj::Simulate::Clock> syncCPU;
//   mjtNum syncSim = 0;

//   // Track previous simulation time to detect UI-triggered resets
//   mjtNum prevSimTime = 0;

//   // run until asked to exit
//   while (!sim_->exitrequest.load())
//   {
//     // sleep for 1 ms or yield, to let main thread run
//     //  yield results in busy wait - which has better timing but kills battery life
//     if (sim_->run && sim_->busywait)
//     {
//       std::this_thread::yield();
//     }
//     else
//     {
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }

//     {
//       // lock the sim mutex during the update
//       const std::unique_lock<std::mutex> lock(*sim_mutex_);

//       // Detect if a reset occurred via the UI (Sync() processed pending_.reset)
//       // This is detected by simulation time jumping backwards to near-zero
//       // The render loop's Sync() calls mj_resetData() which resets time to 0
//       if (mj_model_ && mj_data_ && prevSimTime > 0.1 && mj_data_->time < 0.01)
//       {
//         RCLCPP_DEBUG(get_logger(), "UI reset detected (time jumped from %.3f to %.3f), applying initial state...",
//                      prevSimTime, mj_data_->time);

//         // Restore simulation time before reset_world_state saves it
//         mj_data_->time = prevSimTime;

//         // Apply initial state using common method
//         reset_world_state(true);

//         // Force speed_changed to re-sync timing
//         sim_->speed_changed = true;

//         RCLCPP_INFO(get_logger(), "Successfully applied initial state after UI reset.");
//       }

//       // run only if model is present
//       if (mj_model_)
//       {
//         // Determine the viewer (drag) forces for this outer iteration.
//         //
//         // mjv_updateScene in simulate.cc reads mj_data_->xfrc_applied BEFORE zeroing it, so
//         // plugin forces written here are visible as arrows in the native viewer. To avoid
//         // accumulation across outer iterations we must preserve the viewer-drag portion.
//         // We do this in xfrc_viewer_capture_.
//         //
//         // After each outer iteration we restore mj_data_->xfrc_applied = viewer + plugin and
//         // record it in xfrc_last_written_. We can then combine the desired forces from the plugins
//         // as well as the viewers prior to stepping, without either of them stacking in
//         // undesirable ways.
//         const int nbody6 = 6 * static_cast<int>(mj_model_->nbody);
//         if (std::memcmp(mj_data_->xfrc_applied, xfrc_last_written_.data(), nbody6 * sizeof(mjtNum)) != 0)
//         {
//           // Render thread ran: xfrc_applied was zeroed then drag was applied.
//           mju_copy(xfrc_viewer_capture_.data(), mj_data_->xfrc_applied, nbody6);
//         }
//         // else: render thread did not run; keep the existing xfrc_viewer_capture_.

//         // running (ie, not paused)
//         if (sim_->run)
//         {
//           // If the sim was unpaused while a StepSimulation call was in progress,
//           // abort the remaining pending steps so the service call unblocks cleanly.
//           if (pending_steps_.load() > 0)
//           {
//             RCLCPP_WARN(get_logger(), "Simulation resumed while %u step(s) were still pending; aborting.",
//                         pending_steps_.load());
//             pending_steps_.store(0);
//             steps_interrupted_.store(true);
//             steps_cv_.notify_all();
//           }

//           if (snapshot_refresh_requested_.exchange(false, std::memory_order_acq_rel))
//           {
//             refresh_data_snapshot();
//           }

//           bool stepped = false;

//           // record cpu time at start of iteration
//           const auto startCPU = mj::Simulate::Clock::now();

//           // elapsed CPU and simulation time since last sync
//           const auto elapsedCPU = startCPU - syncCPU;
//           auto elapsedSim = mj_data_->time - syncSim;

//           // Ordinarily the speed factor for the simulation is pulled from the sim UI. However, this is
//           // overridable by setting the "sim_speed_factor" parameter in the hardware info.
//           // If that parameter is set, then we ignore whatever slowdown has been requested from the UI.
//           double speedFactor = sim_speed_factor_ < 0 ? (100.0 / sim_->percentRealTime[sim_->real_time_index]) :
//                                                        (1.0 / sim_speed_factor_);

//           // misalignment condition: distance from target sim time is bigger
//           // than syncmisalign
//           bool misaligned = std::abs(Seconds(elapsedCPU).count() / speedFactor - elapsedSim) > kSyncMisalign;

//           // out-of-sync (for any reason): reset sync times, step
//           if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
//               sim_->speed_changed)
//           {
//             // re-sync
//             syncCPU = startCPU;
//             syncSim = mj_data_->time;
//             sim_->speed_changed = false;

//             apply_staged_control_inputs();
//             // run single step, let next iteration deal with timing
//             mj_step(mj_model_, mj_data_);

            
//             // Publish the per-step control state before the clock tick,
//             // so consumers woken by this tick read state synchronous with that sim time.
//             publish_control_state();
//             publish_clock();

//             const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
//             if (message)
//             {
//               sim_->run = 0;
//               mju::strcpy_arr(sim_->load_error, message);
//             }
//             else
//             {
//               stepped = true;
//               step_count_.fetch_add(1);
//             }
//           }

//           // in-sync: step until ahead of cpu
//           else
//           {
//             bool measured = false;
//             mjtNum prevSim = mj_data_->time;

//             double refreshTime = kSimRefreshFraction / sim_->refresh_rate;

//             // step while sim lags behind cpu and within refreshTime.
//             auto currentCPU = mj::Simulate::Clock::now();
//             while (Seconds((mj_data_->time - syncSim) * speedFactor) < currentCPU - syncCPU &&
//                    currentCPU - startCPU < Seconds(refreshTime))
//             {
//               // measure slowdown before first step
//               if (!measured && elapsedSim)
//               {
//                 sim_->measured_slowdown =
//                     static_cast<float>(std::chrono::duration<double>(elapsedCPU).count() / elapsedSim);
//                 measured = true;
//               }
// // inject noise
// // Use mjVERSION_HEADER and if it is greater than 337 then do one thing or another
// // Needed due to
// // https://github.com/google-deepmind/mujoco/commit/401bf431b8b0fe6e0a619412a607b5135dc4ded4#diff-3dc22ceeebd71304c41d349c6d273bda172ea88ff49c772dbdcf51b9b19bbd33R2943
// #if mjVERSION_HEADER < 337
//               sim_->InjectNoise();
// #else
//               sim_->InjectNoise(-1);
// #endif
//               apply_staged_control_inputs();
//               // call mj_step
//               mj_step(mj_model_, mj_data_);

//               // Publish the per-step control state before the clock tick (see above)
//               publish_control_state();
//               publish_clock();
              
//               publish_wrenches();

//               const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
//               if (message)
//               {
//                 sim_->run = 0;
//                 mju::strcpy_arr(sim_->load_error, message);
//               }
//               else
//               {
//                 stepped = true;
//                 step_count_.fetch_add(1);
//               }

//               // break if reset
//               if (mj_data_->time < prevSim)
//               {
//                 break;
//               }

//               // Update current CPU time for next iteration
//               currentCPU = mj::Simulate::Clock::now();
//             }
//           }

//           // save current state to history buffer
//           if (stepped)
//           {
//             apply_staged_control_inputs();
//             sim_->AddToHistory();
//             update_sim_display();
//           }
//         }

//         // paused
//         else
//         {
//           // Translate keyboard 'S' presses into single pending steps.
//           if (keyboard_step_requested_.exchange(false))
//           {
//             step_diverged_.store(false);
//             pending_steps_.fetch_add(1);
//           }

//           // Record so the next iteration can detect render thread changes, only necessary once
//           // when paused
//           apply_staged_control_inputs();

//           // Execute one pending step per physics loop iteration so the clock publisher
//           // (try_publish) has time to flush between steps, matching play mode behavior.
//           if (pending_steps_.load() > 0)
//           {
//             mj_step(mj_model_, mj_data_);
//             publish_control_state();
//             publish_clock();
            
//             publish_wrenches();

//             const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
//             if (message)
//             {
//               pending_steps_.store(0);
//               step_diverged_.store(true);
//               mju::strcpy_arr(sim_->load_error, message);
//               steps_cv_.notify_all();
//             }
//             else
//             {
//               sim_->AddToHistory();
//               pending_steps_.fetch_sub(1);
//               step_count_.fetch_add(1);
//               steps_cv_.notify_all();
//               update_sim_display();
//             }

//             if (pending_steps_.load() == 0)
//             {
//               sim_->speed_changed = true;
//             }
//           }
//           else
//           {
//             mj_forward(mj_model_, mj_data_);
//             sim_->speed_changed = true;
//             update_sim_display();
//           }

//           // Keep the snapshots in sync while paused so reads reflect steps and UI edits
//           if (snapshot_refresh_requested_.exchange(false, std::memory_order_acq_rel))
//           {
//             refresh_data_snapshot();
//           }
//           publish_control_state();
//         }

//         // Update previous simulation time for next iteration
//         if (mj_data_)
//         {
//           prevSimTime = mj_data_->time;
//         }
//       }
//     }  // release std::lock_guard<std::mutex>
//   }
// }





    void GetSimData::print_all_joint_torques(const mjModel* m, mjData* d) {
       
        std::cout << "\n================ TOTAL JOINT TORQUE BREAKDOWN ================\n";
        std::cout << std::left 
                << std::setw(25) << "Joint Name"
                << std::setw(15) << "Internal (Nm)"
                << std::setw(15) << "External (Nm)"
                << std::setw(15) << "Total Net (Nm)" << "\n";
        std::cout << "--------------------------------------------------------------\n";

        // Loop through all joints in the model (m->njnt is total number of joints)
        for (int jnt_id = 0; jnt_id < m->njnt; ++jnt_id) {
            
            // 1. Fetch Joint Name
            const char* name = mj_id2name(m, mjOBJ_JOINT, jnt_id);
            std::string jnt_name = name ? name : ("joint_" + std::to_string(jnt_id));

            // 2. Get degree-of-freedom offset and type
            int dof_adr = m->jnt_dofadr[jnt_id];
            int jnt_type = m->jnt_type[jnt_id];

            // 3. Determine how many DoFs this joint has 
            // (hinge/slide = 1, ball = 3, free = 6)
            int dof_num = 1;
            if (jnt_type == mjJNT_BALL) {
                dof_num = 3;
            } else if (jnt_type == mjJNT_FREE) {
                dof_num = 6;
            }

            // 4. Iterate over each DoF for this joint
            for (int d_idx = 0; d_idx < dof_num; ++d_idx) {
                int dof = dof_adr + d_idx;

                // Extract force components
                mjtNum actuator   = d->qfrc_actuator[dof];   // Motor forces
                mjtNum passive    = d->qfrc_passive[dof];    // Damping/springs/fluid
                mjtNum applied    = d->qfrc_applied[dof];    // External applied forces
                mjtNum constraint = d->qfrc_constraint[dof]; // Contact & joint limit forces
                mjtNum bias       = d->qfrc_bias[dof];       // Gravity, Coriolis, centrifugal

                // Groupings
                mjtNum internal_torque  = actuator + passive + applied;
                mjtNum external_torque  = constraint;
                mjtNum total_net_torque = internal_torque + external_torque - bias;
                (void)total_net_torque;
                // Handle multi-DoF joint labeling
                std::string label = jnt_name;
                if (dof_num > 1) {
                    label += " [dof " + std::to_string(d_idx) + "]";
                }

                // // Print summary
                // std::cout << std::left 
                //           << std::setw(25) << label
                //           << std::setw(15) << internal_torque
                //           << std::setw(15) << external_torque
                //           << std::setw(15) << total_net_torque << "\n";
                // Print summary
                std::cout << constraint << "\n";

            }
        }
        std::cout << "==============================================================\n\n";
    }



    void GetSimData::publish_wrenches(){

        std::lock_guard<std::recursive_mutex> lock(this->getMutex());

        // model() and data() sono funzioni pubbliche, e posso accedere agli oggetti
        
        mjModel* model = this->model();
        mjData* data = this->data();
        
        int body_id = mj_name2id(model, mjOBJ_BODY, "base_link");

        publish_odometry(body_id, model, data);

        getGroundContactWrench(body_id, model, data);
        publish_imu_data_kinematics(body_id, model, data);

        const mjtNum* wrench = data->cfrc_int + 6 * body_id;

        mujoco_ros2_control_msgs::msg::BodyWrench msg;
        
        msg.header.stamp = node_->now();
        msg.header.frame_id = "base_link";

        msg.body_name = "base_link";

        msg.wrench.force.x = wrench[3];
        msg.wrench.force.y = wrench[4];
        msg.wrench.force.z = wrench[5];

        msg.wrench.torque.x = wrench[0];
        msg.wrench.torque.y = wrench[1];
        msg.wrench.torque.z = wrench[2];

        wrench_pub_->publish(msg);
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
        msg.angular_velocity.x = cvel[0];
        msg.angular_velocity.y = cvel[1];
        msg.angular_velocity.z = cvel[2];

        // 3. Linear acceleration from 6D spatial acceleration vector (cacc: [ang_acc..., lin_acc...])
        const mjtNum* cacc = data->cacc + 6 * body_id;
        msg.linear_acceleration.x = cacc[3];
        msg.linear_acceleration.y = cacc[4];
        msg.linear_acceleration.z = cacc[5]; // Add gravity vector if simulating physical accelerometer

        imu_pub_->publish(msg);
    }



    void GetSimData::getGroundContactWrench(int body_id, const mjModel* model, mjData* data ){

    mujoco_ros2_control_msgs::msg::ContactForces msg;

    msg.header.stamp = node_->now(); 
    msg.header.frame_id = "base_link";

    // 2. Get the target body's Center of Mass (COM) position in World Frame
    const mjtNum* body_com = &data->xipos[3 * body_id];

    for (int i = 0; i < data->ncon; ++i) {

        const mjContact& con = data->contact[i];
        //con.geom1 and con.geom2 are the 2 contact geometries

        int body1 = model->geom_bodyid[con.geom1];
        int body2 = model->geom_bodyid[con.geom2];

        const char* name1 = mj_id2name(model, mjOBJ_BODY, body1);
        const char* name2 = mj_id2name(model, mjOBJ_BODY, body2);

        msg.body1_names.push_back(name1 ? name1 : "unnamed_body_" + std::to_string(body1));
        msg.body2_names.push_back(name2 ? name2 : "unnamed_body_" + std::to_string(body2));

        geometry_msgs::msg::Point pos;
        pos.x = con.pos[0];
        pos.y = con.pos[1];
        pos.z = con.pos[2];
        msg.contact_positions.push_back(pos);

        // 6D force in contact frame: [normal, friction_x, friction_y, torque_r, torque_p, torque_y]
        mjtNum c_force[6];
        mj_contactForce(model, data, i, c_force);

        // Sign flip depending on whether body_id is body1 or body2 (Newton's 3rd Law)
        double sign = (body1 == body_id) ? 1.0 : -1.0;


        const mjtNum* R_world_baselink = &data->xmat[9 * body_id];
        // Transform 3D linear force to World Frame: F_world = con.frame^W_c * c_force[0..2]. con.frame is a 3x3 rotation matrix that describe the orientation of the contact frame
        mjtNum f_world[3];
        mju_mulMatTVec(f_world, con.frame, c_force, 3, 3);

        mjtNum f_base_link[3];
        mju_mulMatTVec(f_base_link, R_world_baselink, f_world, 3, 3);


        // Scale by sign
        f_base_link[0] *= sign;
        f_base_link[1] *= sign;
        f_base_link[2] *= sign;

        geometry_msgs::msg::Vector3 f_msg;
        f_msg.x = f_base_link[0] * sign;
        f_msg.y = f_base_link[1] * sign;
        f_msg.z = f_base_link[2] * sign;
        msg.forces.push_back(f_msg);

        // Transform 3D contact torsional torque to World Frame
        mjtNum t_contact_world[3];
        mju_mulMatTVec(t_contact_world, con.frame, &c_force[3], 3, 3);
        //The third argument needs the address of an element, here I am giving the address of the third element of c_force, that is the torsional contact forces

        // Calculate moment arm: r = contact_position - body_COM
        mjtNum r[3] = {
        con.pos[0] - body_com[0],
        con.pos[1] - body_com[1],
        con.pos[2] - body_com[2]
        };

        // Calculate induced torque about COM: tau_moment = r x F
        mjtNum t_moment_world[3];
        mju_cross(t_moment_world, r, f_base_link);

        geometry_msgs::msg::Vector3 t_msg;
        t_msg.x = t_contact_world[0] * sign;// + t_moment_world[0];
        t_msg.y = t_contact_world[1] * sign;// + t_moment_world[1];
        t_msg.z = t_contact_world[2] * sign;// + t_moment_world[2];
        msg.torques.push_back(t_msg);

    }
        
    contact_forces_pub_->publish(msg);

    }




    // void GetSimData::publish_actuator_wrenches()
    // {
    //     std::lock_guard<std::recursive_mutex> lock(this->getMutex());

    //     int body_id = mj_name2id(sim_->getModel(), mjOBJ_BODY, "base_link");
    //     if (body_id < 0)
    //     {
    //     RCLCPP_ERROR_THROTTLE(
    //         node_->get_logger(), *node_->get_clock(), 2000,
    //         "Body 'base_link' not found in MuJoCo model.");
    //     return;
    //     }

    //     int nv = sim_->getModel()->nv; // Total degrees of freedom

    //     // Allocations for Jacobian matrices (Linear and Angular)
    //     std::vector<double> jacp(3 * nv, 0.0); // 3 x nv linear Jacobian, but allocated as a whole row vector
    //     std::vector<double> jacr(3 * nv, 0.0); // 3 x nv rotational Jacobian

    //     // Compute 6D Jacobian at the body frame center of mass
    //     mj_jacBody(
    //         sim_->getModel(),
    //         sim_->getData(),
    //         jacp.data(),
    //         jacr.data(),
    //         body_id);

    //     // Get the generalized forces generated strictly by ALL actuators
    //     const mjtNum* qfrc_act = sim_->getData()->qfrc_actuator;

    //     double force[3] = {0.0, 0.0, 0.0};
    //     double torque[3] = {0.0, 0.0, 0.0};

    //     // Multiply J_T * qfrc_actuator to map joint-space forces to Cartesian 3D Force & Torque
    //     for (int i = 0; i < nv; ++i)
    //     {
    //     force[0] += jacp[i] * qfrc_act[i];
    //     force[1] += jacp[nv + i] * qfrc_act[i];
    //     force[2] += jacp[2 * nv + i] * qfrc_act[i];

    //     torque[0] += jacr[i] * qfrc_act[i];
    //     torque[1] += jacr[nv + i] * qfrc_act[i];
    //     torque[2] += jacr[2 * nv + i] * qfrc_act[i];
    //     }

    //     mujoco_ros2_control_msgs::msg::BodyWrench msg;
    //     msg.header.stamp = node_->now();
    //     msg.header.frame_id = "world";
    //     msg.body_name = "base_link";

    //     msg.wrench.force.x = force[0];
    //     msg.wrench.force.y = force[1];
    //     msg.wrench.force.z = force[2];

    //     msg.wrench.torque.x = torque[0];
    //     msg.wrench.torque.y = torque[1];
    //     msg.wrench.torque.z = torque[2];

    //     wrenches_from_actuators_pub_->publish(msg);
    // }


    void GetSimData::publish_odometry(int body_id, const mjModel* model, mjData* data )
    {

        rclcpp::Time current_time = node_->now();

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

        // Pointer to global rotation matrix xmat (3x3 row-major)
        mjtNum* R = &data->xmat[body_id * 9];
        // Multiply the 1st column by -1 (indices 0, 3, and 6)-
        // Faccio questo perchè sospetto che il base_link e il world_link abbiano l'asse x in direzioni opposte e quando il camion va 5m/s mi trovo che dice che sta a -5m/s
        R[0] *= -1.0;
        R[3] *= -1.0;
        R[6] *= -1.0;
        // 2. Rotate World -> Body frame: v_body = R^T * v_world
        mjtNum body_lin_vel[3];
        mjtNum body_ang_vel[3];

        // mju_mulMatTVec multiplies Transpose(R) * world_lin_vel
        mju_mulMatTVec(body_lin_vel, R, world_lin_vel, 3, 3);
        mju_mulMatTVec(body_ang_vel, R, world_ang_vel, 3, 3);


        // -------------------------------------------------------------
        // Populate Odometry Message
        // -------------------------------------------------------------
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

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
        odom_msg.twist.twist.linear.x = body_lin_vel[0];
        odom_msg.twist.twist.linear.y = body_lin_vel[1];
        odom_msg.twist.twist.linear.z = body_lin_vel[2];

        odom_msg.twist.twist.angular.x = body_ang_vel[0];
        odom_msg.twist.twist.angular.y = body_ang_vel[1];
        odom_msg.twist.twist.angular.z = body_ang_vel[2];

        // Publish Odometry
        odom_pub_->publish(odom_msg);

        // -------------------------------------------------------------
        // Broadcast TF (odom -> base_footprint)
        // -------------------------------------------------------------
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_footprint";

        tf_msg.transform.translation.x = pos_x;
        tf_msg.transform.translation.y = pos_y;
        tf_msg.transform.translation.z = pos_z;

        tf_msg.transform.rotation.w = quat_w;
        tf_msg.transform.rotation.x = quat_x;
        tf_msg.transform.rotation.y = quat_y;
        tf_msg.transform.rotation.z = quat_z;

        tf_broadcaster_->sendTransform(tf_msg);
    }
    
}