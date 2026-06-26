#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_matrix

from .gazebo_model import Callbacks
from . import robot
from observer.utils.data_plotter import DataPlotter
from .states import state as s

from geometry_msgs.msg import Twist


from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import GetParameters


"""dot_x1 = x2                      x = [x1] -> [x,y,roll,yaw]
    dot_x2 = f(x,u)                     [x2] -> [vx,vy,dot_roll,dot_yaw]"""
class TruckStateObserver(Node):

    def __init__(self):
        super().__init__('truck_state_observer')

        self.group = ReentrantCallbackGroup()

        self.declare_parameter('scale_tanh', 10.0)
        self.scale_tanh = self.get_parameter('scale_tanh').value

        self.declare_parameter("des_vel_x", 5.0)
        self.des_vel_x = self.get_parameter('des_vel_x').value

        self.declare_parameter("des_omega_z", 0.5) #giving a desired angular vel prevent the velocity to reach a steady state
        self.des_omega_z = self.get_parameter('des_omega_z').value
        
        # Friend Function
        self.truck = robot.robot()
        self.cb = Callbacks(self)
        self.plotter = DataPlotter("results")

        self.UseGazeboSim = False
        if self.get_parameter("use_sim_time").value:
            self.fixed_dt = 1e-3 #lo rendi abbastanza basso da usare il tempo di gazebo
            self.UseGazeboSim = True

            self.timer_gazebo = self.create_timer(
                self.fixed_dt, 
                self.GazeboControl,
                callback_group=self.group
            )

            self.gz_pub_ = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )

            # To get the parameter from the truck_controller
            self.steering_angle_sub = self.create_subscription(
                Float64MultiArray,
                '/steering_controller/commands',
                self.steering_callback,   # callback function
                10,
                callback_group=self.group
            )

            self.L = np.zeros(4)
            self.param_client = self.create_client(
                GetParameters,
                '/truck_kinematic_control/get_parameters'
            )

            request = GetParameters.Request()
            request.names = ['L1', 'L2', 'L3', 'L4']
            print("HIIII")
            future = self.param_client.call_async(request)
            future.add_done_callback(self.get_L_parameters)

            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data', #50Hz
                self.cb.imu_callback,
                10
            )

            self.odom_sub = self.create_subscription(
                Odometry,
                '/odometry', #20Hz
                self.cb.odom_callback,
                10
            )
        else:
            self.fixed_dt = 1e-2 #The observer should run at 10ms

        self.delta = np.zeros(2, dtype='float32')
        self.rotation_body2world = np.zeros((3,3), dtype='float32')
        
        num_states = 8
        self.state = np.zeros(num_states, dtype="float32")
        self.observed_state = np.zeros(num_states, dtype="float32")
        self.estimate_error_x1 = np.zeros(4, dtype="float32")

        self.e_int = np.zeros(2,dtype="float32")
        self.previous_e = np.zeros(2,dtype="float32")

        # The controller frequency will always depend on this, and so the observer if it runs in the callback
        self.timer_observer = self.create_timer(
            self.fixed_dt, 
            self.ModelSimulator,
            callback_group=self.group
        )

        self.counter = 0
        self.early = None

        self.time_data = []
        self.state_data = []
        self.observed_data = []
        self.sim_time = 0.0


    def GazeboControl(self):
        msg = Twist()
        # Linear velocity (m/s)
        msg.linear.x = self.des_vel_x
        msg.linear.y = 0.0
        msg.angular.z = self.des_omega_z

        self.gz_pub_.publish(msg)


    def ModelSimulator(self):
 
        dt = self.fixed_dt

        if(not self.UseGazeboSim):
            u = self.PdController(dt = dt)
            Fx, Mz = u
            #self.state += self.truck.dynamics(self.state, Fx, Mz, add_disturb = True)*dt
            self.state = self.rk4_step(self.state, Fx, Mz, dt, dist = True)
        else:
            self.state = self.cb.gazebo_state
            # For the observer, to use the Urdf parameter for L and delta and not the one defined in simulink
            self.truck.getFurtherParameterFromUrdf(self.L, self.delta)

        self.estimate_error_x1 = self.state[:4] - self.observed_state[:4]


        dot_observed_state = self.truck.dynamics(self.observed_state, Fx, Mz, add_disturb = False)

        self.HighOrderObserver(dot_observed_state, dt)



    def HighOrderObserver(self, dot_observed_state, dt):

        p = 0.05
        alpha = 2*np.abs(dot_observed_state[:4]) + 150.0
        lambdaa = np.sqrt(2/(alpha - 2*np.abs(dot_observed_state[:4])))* ( (alpha + 2*np.abs(dot_observed_state[:4]))*(1+p))/(1-p) + 10
        delta = np.sqrt(np.abs(self.estimate_error_x1))
        correction_term = lambdaa*delta*np.tanh(self.scale_tanh*self.estimate_error_x1)

        dot_observed_state[:4] = dot_observed_state[:4] + correction_term
        dot_observed_state[4:] = dot_observed_state[4:] + alpha*np.tanh(self.scale_tanh*self.estimate_error_x1)
        
        self.observed_state += dot_observed_state*dt
        
        # Save variables for Plotting
        self.sim_time += dt
        self.time_data.append(self.sim_time)
        self.state_data.append(
            self.state.copy()
        )
        self.observed_data.append(
            self.observed_state.copy()
        )

        st = self.state
        obs = self.observed_state
        self.counter+=1
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"real (estimated)\n"
                f"Pos : x={st[s.X]:7.3f} ({obs[s.X]:7.3f})  "
                f"y={st[s.Y]:7.3f} ({obs[s.Y]:7.3f})\n"
                f"Ang : r={st[s.ROLL]:7.3f} ({obs[s.ROLL]:7.3f})  "
                f"ψ={st[s.YAW]:7.3f} ({obs[s.YAW]:7.3f})\n"
                f"Vel : vx={st[s.VX]:7.3f} ({obs[s.VX]:7.3f})  "
                f"vy={st[s.VY]:7.3f} ({obs[s.VY]:7.3f})\n"
                f"Rate: wx={st[s.WX]:7.3f} ({obs[s.WX]:7.3f})  "
                f"wz={st[s.WZ]:7.3f} ({obs[s.WZ]:7.3f})\n"
                f"dt={dt:.4f}"
            )


    def PdController(self, dt):

        if(not self.UseGazeboSim):
            roll = self.state[s.ROLL]
            pitch = 0.0
            yaw = self.state[s.YAW]

            self.rotation_body2world = R.from_euler(
                'xyz',
                [roll, pitch, yaw]
            ).as_matrix().T

        Kp = np.array([100000, 2000000])
        Ki = np.array([100, 100.0])
        Kd = np.array([100, 10])

        vx_des, _, des_omega_z =  np.array([self.des_vel_x, 0, self.des_omega_z])
        
        control_error = np.array([
            vx_des - self.state[s.VX],
            des_omega_z - self.state[s.WZ]
        ])

        derivative_term = Kd*((control_error - self.previous_e)/dt)
        integral_term = Ki*self.e_int

        u = Kp*control_error + integral_term  
    
        self.previous_e = control_error
        self.e_int += control_error*dt

        return u
    

    # def PdController(self, dt):
    #     # 1. Coordinate transformation (if required elsewhere, kept for compatibility)
    #     if not self.UseGazeboSim:
    #         roll = self.state[s.ROLL]
    #         pitch = 0.0
    #         yaw = self.state[s.YAW]

    #         self.rotation_body2world = R.from_euler(
    #             'xyz',
    #             [roll, pitch, yaw]
    #         ).as_matrix().T

    #     # 2. Controller Gains
    #     Kp = np.array([100000.0, 1000.0])
    #     Ki = np.array([100.0, 1.0])
    #     Kd = np.array([100.0, 10.0])

    #     # 3. Target States
    #     vx_des = self.des_vel_x
    #     des_omega_z = self.des_omega_z
        
    #     # 4. Compute Tracking Error
    #     control_error = np.array([
    #         vx_des - self.state[s.VX],
    #         des_omega_z - self.state[s.WZ]
    #     ])

    #     # 5. Update Integral Term with Windup Protection (Clamping)
    #     self.e_int += control_error * dt
        
    #     # Simple anti-windup: limit max integral error contribution
    #     # Adjust the max limits [vx_int_max, wz_int_max] based on your vehicle
    #     max_int_limit = np.array([500.0, 50.0]) 
    #     self.e_int = np.clip(self.e_int, -max_int_limit, max_int_limit)

    #     # 6. Compute PID Terms
    #     proportional_term = Kp * control_error
    #     integral_term = Ki * self.e_int
    #     derivative_term = Kd * ((control_error - self.previous_e) / dt)

    #     # 7. Total Control Output (Fixed: added derivative_term)
    #     u = proportional_term + integral_term + derivative_term
        
    #     # 8. Save current error for the next time step
    #     self.previous_e = control_error

    #     return u

    def rk4_step(self, state, Fx, Mz, dt, dist):

        k1 = self.truck.dynamics(state, Fx, Mz, dist)
        k2 = self.truck.dynamics(state + 0.5*dt*k1, Fx, Mz, dist)
        k3 = self.truck.dynamics(state + 0.5*dt*k2, Fx, Mz, dist)
        k4 = self.truck.dynamics(state + dt*k3, Fx, Mz, dist)

        return state + dt/6.0 * (k1 + 2*k2 + 2*k3 + k4)



    def AtEnd(self):

        state = np.array(self.state_data)
        observed = np.array(self.observed_data)
        time = np.array(self.time_data)
        
        self.plotter.plot(
            time,
            state[:, s.VX],
            observed[:, s.VX],
            ylabel="vx [m/s]",
            filename="velocities.png"
        )

        self.plotter.plot(
            time,
            state[:, s.WZ],
            observed[:, s.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

 
    def steering_callback(self, msg: Float64MultiArray):
        # Example: first steering command
        if len(msg.data) > 0:
            self.delta = msg.data

    def get_L_parameters(self, future):
        try:
            result = future.result()

            self.L = [
                result.values[0].double_value,
                result.values[1].double_value,
                result.values[2].double_value,
                result.values[3].double_value,
            ]

        except Exception as e:
            self.get_logger().error(
                f"Failed to get L parameters: {e}"
            )
def main(args=None):

    rclpy.init(args=args)

    node = TruckStateObserver()
    try:
        if node.UseGazeboSim:
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(node)
            executor.spin()  # Handled by the outer try-except now
        else:
            rclpy.spin(node) # Handled by the outer try-except now

    except KeyboardInterrupt:
        # This catches Ctrl+C cleanly for BOTH Gazebo and non-Gazebo modes!
        node.get_logger().info('Shutting down observer node cleanly...')
        
    finally:
        # Clean up the executor if it was initialized
        if node.UseGazeboSim and 'executor' in locals():
            executor.shutdown()
            
        # Run your custom end routine and destroy the node
        node.AtEnd()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()