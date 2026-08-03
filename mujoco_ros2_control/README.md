## Acknowledgements

This repository is a fork / custom extension of the original [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) package. Maintainers and authors of the original repository are listed in the package manifests (package.xml) inside mujoco_ros2_control package.

# Modifications to do in mujoco_simulation.hpp

  Inside the mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control you must to:

  - Change the scope of these variables from private to protected, in order to make it usable also from the child class

  ``` cpp
  protected:
  /**
   * @brief Loops the physics simulation until asked to terminate.
   */
  virtual void physics_loop();

  std::unique_ptr<mujoco::Simulate> sim_;

  rclcpp::Node::SharedPtr node_;
  ```

  - Make the initialize function a virtual class, in order to use the one of the child that will override this:

    ``` cpp
  virtual bool initialize(rclcpp::Node::SharedPtr node, const std::string& model_path, const std::string& mujoco_model_topic,
                  double sim_speed_factor, bool headless);
  ```

  - Add all the methods you are implementing in the child class as virtual *abstract* methods


  ``` cpp
    virtual void publish_wrenches() = 0;

    //void publish_actuator_wrenches() = 0;

    virtual void print_all_joint_torques(const mjModel* m, mjData* d) = 0;

    virtual void publish_odometry(int body_id, const mjModel* model, mjData* data) = 0;

    virtual void getGroundContactWrench(int body_id, const mjModel* model, mjData* data) = 0;

    virtual void publish_imu_data_kinematics(int body_id, const mjModel* model, mjData* data)=0;
  ```

# Modifications to do in mujoco_simulation.cpp


  Call the main method of your derived class inside the physics_loop() function


# Modifications to do in mujoco_system_interface.cpp

Here you will exploit the late binding, because inside the mujoco_system_interface.hpp you have declared:

  ``` cpp

  std::unique_ptr<MujocoSimulation> simulation_;

  ``` 

  Where *MujocoSimulation* is the parent class. Inside mujoco_system_interface.cpp you have to change this line as follow:

  ``` cpp
    simulation_ = std::make_unique<mujoco_ros2_control_truck::GetSimData>();
  ``` 

  In this way you are creating a late binding because you are initializing an instance of an objet of the parent class, with the pointer of the child class. 
  According the polymorphism, the parent instance (that is `simulation_`) will call the method of the parent class, expect for the method that has been defined
  as `virtual`, that is all the one you define in the child class. This is because, theparent object has been initialized with a pointer of the child class