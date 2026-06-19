#ifndef GZ_SIM_SYSTEMS_TRUCK_PLUGIN_HH
#define GZ_SIM_SYSTEMS_TRUCK_PLUGIN_HH

#include <memory>
#include <gz/sim/System.hh>



namespace gz{
    namespace sim{
        inline namespace GZ_SIM_VERSION_NAMESPACE{
            namespace systems{

                class TruckPluginPrivate;

                /// \brief Truck controller which can be attached to a model of a Truck with 4 or more motorized wheels, and 2 front steering axes. 
                ///
                /// ## System Parameters
                ///
                /// - `<driven_wheels>`: Number of driven wheels. Supported values are 4 and 8. Default is 4. 
                ///
                /// - `<central_wheel_base>`: Distance between two central wheel axes, in meters. This
                /// element is optional, although it is recommended to be included with an
                /// appropriate value. The default value is 1.0m.
                ///
                /// - `<front_wheel_base>`: Distance between two front wheel axes, in meters. This
                /// element is optional, although it is recommended to be included with an
                /// appropriate value. The default value is 1.0m.
                ///
                /// - `<rear_wheel_base>`: Distance between two rear wheel axes, in meters. This
                /// element is optional, although it is recommended to be included with an
                /// appropriate value. The default value is 1.0m.
                ///
                /// - `<steer_p_gain>`: Float used to control the steering angle P gain.
                ///
                /// - `<wheel_radius>`: Wheel radius in meters. This element is optional,
                /// although it is recommended to be included with an appropriate value. The
                /// default value is 0.5m.
                ///
                /// - `<min_velocity>`: Minimum velocity [m/s], usually <= 0.
                /// - `<max_velocity>`: Maximum velocity [m/s], usually >= 0.
                /// - `<min_acceleration>`: Minimum acceleration [m/s^2], usually <= 0.
                /// - `<max_acceleration>`: Maximum acceleration [m/s^2], usually >= 0.
                ///
                /// - `<rear_left_joint>`: Name of a joint that controls a left wheel. This
                /// element can appear multiple times, and must appear twice.
                ///
                /// - `<rear_right_joint>`: Name of a joint that controls a right wheel. This
                /// element can appear multiple times, and must appear twice.
                ///
                /// - `<front_left_1_steering_joint>`: Name of the steering joint for the left
                ///   wheel of the first front steering axis. This element must appear once.
                ///
                /// - `<front_right_1_steering_joint>`: Name of the steering joint for the right
                ///   wheel of the first front steering axis. This element must appear once.
                ///
                /// - `<front_left_2_steering_joint>`: Name of the steering joint for the left
                ///   wheel of the second front steering axis. This element must appear once.
                ///
                /// - `<front_right_2_steering_joint>`: Name of the steering joint for the right
                ///   wheel of the second front steering axis. This element must appear once.
                ///
                /// - `<front_left_1_drive_joint>`: Name of the drive joint for the left wheel
                ///   of the first front drive axis. This element must appear once.
                ///
                /// - `<front_right_1_drive_joint>`: Name of the drive joint for the right wheel
                ///   of the first front drive axis. This element must appear once.
                ///
                /// - `<front_left_2_drive_joint>`: Name of the drive joint for the left wheel
                ///   of the second front drive axis. This element must appear once.
                ///
                /// - `<front_right_2_drive_joint>`: Name of the drive joint for the right wheel
                ///   of the second front drive axis. This element must appear once.
                ///
                /// - `<steering_limit>`: Value to limit steering to.  Can be calculated by
                /// measuring the turning radius and wheel_base of the real vehicle.
                /// steering_limit = asin(wheel_base / turning_radius)
                /// The default value is 0.5 radians.
                ///
                /// - `<odom_publish_frequency>`: Odometry publication frequency. This
                /// element is optional, and the default value is 50Hz.
                ///
                /// - `<odom_topic>`: Custom topic on which this system will publish odometry
                /// messages. This element if optional, and the default value is
                /// `/model/{name_of_model}/odometry`.
                ///
                /// - `<tf_topic>`: Custom topic on which this system will publish the
                /// transform from `frame_id` to `child_frame_id`. This element is optional,
                /// and the default value is `/model/{name_of_model}/tf`.
                ///
                /// - `<frame_id>`: Custom `frame_id` field that this system will use as the
                /// origin of the odometry transform in both the `<tf_topic>`
                /// `gz.msgs.Pose_V` message and the `<odom_topic>`
                /// `gz.msgs.Odometry` message. This element if optional, and the
                /// default value is `{name_of_model}/odom`.
                ///
                /// - `<child_frame_id>`: Custom `child_frame_id` that this system will use as
                /// the target of the odometry transform in both the `<tf_topic>`
                /// gz.msgs.Pose_V message and the `<odom_topic>`
                /// gz.msgs.Odometry message. This element is optional,
                /// and the default value is `{name_of_model}/{name_of_link}`.
                ///
                class TruckPlugin: public System, //Classe base da cui si eredita ogni plugin
                                   public ISystemConfigure, //classe per configurare il plugin all'avvio
                                   public ISystemPreUpdate, //classe per eseguire codice ad ogni step della simulazione prima della fisica (Calcolo degli angoli di steering)
                                   public ISystemPostUpdate //classe per eseguire codice ad ogni step della simulazione dopo la fisica (Calcolo dell'odometria, pubblicare i dati)
                {
                    public:
                        TruckPlugin();
                        ~TruckPlugin() override=default;
                        void Configure(const Entity &entity,
                                       const std::shared_ptr<const sdf::Element> &sdf, 
                                       EntityComponentManager &ecm, 
                                       EventManager &eventMgr) override;

                        void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;

                        void PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm) override;

                    private:
                        std::unique_ptr<TruckPluginPrivate> dataPtr;
                };

            }//systems
        }//GZ_SIM_VERSION_NAMESPACE
    }//sim
}//gz
  
#endif