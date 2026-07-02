#!/usr/bin/env python3
import sys
import argparse

# 1. Parse command line arguments before ROS starts
parser = argparse.ArgumentParser(description="Truck State Observer Node")
parser.add_argument('--num-states', type=int, default=6, choices=[6, 8],
                    help='Number of system states to observe (6 or 8)')
# Extract our custom args while leaving ROS internal arguments intact
parsed_args, ros_args = parser.parse_known_args()
num_states = parsed_args.num_states

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_matrix

from .gazebo_model import GazeboCallback
from . import robot
from observer.utils.data_plotter import DataPlotter

if num_states==6:
    from observer.utils.states import obs_state as s
else:
    from observer.utils.states import state as s

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
        
        self.Gazebo = GazeboCallback(self)
        
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
            future = self.param_client.call_async(request)
            future.add_done_callback(self.get_L_parameters)

            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data', #50Hz
                self.Gazebo.imu_callback,
                10
            )

            self.odom_sub = self.create_subscription(
                Odometry,
                '/odometry', #20Hz
                self.Gazebo.odom_callback,
                10
            )
        else:
            self.fixed_dt = 1e-2 #The observer should run at 10ms

        self.delta = np.zeros(2, dtype='float32')
        self.rotation_body2world = np.zeros((3,3), dtype='float32')
        
        # State terms
        self.state = np.zeros(num_states, dtype=np.float64)
        self.output = np.zeros(6, dtype="float32")

        # Observer terms
        self.observed_state = np.zeros(num_states, dtype="float32")
        
        self.observed_output = np.zeros(6, dtype="float32")
        
        self.jacobian = np.zeros((6,6), dtype='float32')

        # PID Terms
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
        self.w = np.zeros(5, dtype=np.float64)

    def GazeboControl(self):
        msg = Twist()
        # Linear velocity (m/s)
        msg.linear.x = self.des_vel_x
        msg.linear.y = 0.0
        msg.angular.z = self.des_omega_z

        self.gz_pub_.publish(msg)


    def ModelSimulator(self):
 
        dt = self.fixed_dt
        u = self.PdController(dt = dt)
        Fx, Mz = u

        dot_observed_state = None

        if(num_states == 8):
            if(not self.UseGazeboSim):
                #self.state += self.truck.dynamics(self.state, Fx, Mz, add_disturb = True)*dt
                self.state = self.rk4_step(self.state, Fx, Mz, dt, dist = True)
            else:
                self.state = self.Gazebo.gazebo_full_state

            dot_observed_state = self.truck.dynamics(self.observed_state, Fx, Mz, add_disturb = False)
            
            self.HighOrderObserver(dot_observed_state, dt)

        else:
            if(not self.UseGazeboSim):

                dot_x, J_x, h_meas, _ = self.truck.calculate_dynamics(self.state, Fx, Mz, True, dt+self.sim_time)
                
                self.state += dot_x*dt

                self.output = h_meas

            else:

                self.output = self.Gazebo.gazebo_output
            
                self.truck.getFurtherParameterFromUrdf(self.L, self.delta)

            dot_observed_state, _, h_hat_meas, J_h = self.truck.calculate_dynamics(self.observed_state, Fx, Mz, False, dt+self.sim_time)

            # Remove the 6th row (index 5) along axis 0
            matrix_reduced = np.delete(J_h, 5, axis=0)
            # Remove the 5th column (index 4) along axis 1
            matrix_reduced = np.delete(matrix_reduced, 4, axis=1)

            self.jacobian = matrix_reduced

            self.observed_output = h_hat_meas
            self.JacobianObserver(dot_observed_state, dt)   
            # For the observer, to use the Urdf parameter for L and delta and not the one defined in simulink
            
        self.sim_time += dt
        self.time_data.append(self.sim_time)
        self.state_data.append(self.state.copy())
        self.observed_data.append(self.observed_state.copy())



    def HighOrderObserver(self, dot_observed_state, dt):

        p = 0.05
        
        estimated_error = self.state[:4] - self.observed_state[:4]

        alpha = 2*np.abs(dot_observed_state[:4]) + 150.0
        
        lambdaa = np.sqrt(2/(alpha - 2*np.abs(dot_observed_state[:4])))* ( (alpha + 2*np.abs(dot_observed_state[:4]))*(1+p))/(1-p) + 10
        
        root_abs_error = np.sqrt(np.abs(estimated_error))
        
        correction_term = lambdaa*root_abs_error*np.tanh(self.scale_tanh*estimated_error)

        dot_observed_state[:4] = dot_observed_state[:4] + correction_term
        dot_observed_state[4:] = dot_observed_state[4:] + alpha*np.tanh(self.scale_tanh*estimated_error)
        
        self.observed_state += dot_observed_state*dt
        
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
    
    def JacobianObserver(self, dot_observed_state, dt):

        J_inv = np.linalg.inv(self.jacobian)

        alpha = 50

        beta = 1.5

        estimated_error = self.output - self.observed_output
        # Take the estimate error on the first 5 output (I am excluding the vx)
        estimated_error = estimated_error[:5]

        root_abs_error = np.sqrt(np.abs(estimated_error))

        # 2. Overwrite just the specific indices using vectorized assignment
        root_abs_error[2] = np.abs(estimated_error[2]) ** (2/3)
        root_abs_error[3] = np.abs(estimated_error[3]) ** (2/3)

        sign_error = np.tanh(self.scale_tanh*estimated_error)

        self.w += beta*sign_error
        
        correction_term = alpha * root_abs_error * sign_error + self.w

        #Remove the 5th elements
        dot_observed_state_reduced = np.delete(dot_observed_state, 4)

        state_rate_reduced = dot_observed_state_reduced + J_inv @ correction_term
        # Re-insert a 0.0 value at index 4 so it matches the 6-element layout of self.observed_state

        state_rate_6d = np.insert(state_rate_reduced, 4, 0.0)

        self.observed_state += state_rate_6d * dt

        self.observed_state[4] = self.output[5]
            
        st = self.state
        obs = self.observed_state
        self.counter+=1
        if self.counter % 100 == 0:
            self.get_logger().info(
            f"real (estimated)\n"
            f"Ang : r={st[s.ROLL]:7.3f} ({obs[s.ROLL]:7.3f})\n "
            f"Vel : vx={st[s.VX]:7.3f} ({obs[s.VX]:7.3f}) "
            f"vy={st[s.VY]:7.3f} ({obs[s.VY]:7.3f})\n"
            f"Rate: wx={st[s.WX]:7.3f} ({obs[s.WX]:7.3f})  "
            f"wz={st[s.WZ]:7.3f} ({obs[s.WZ]:7.3f})\n"
            f"dt={dt:.4f}"
        )


    def PdController(self, dt):

        # if(not self.UseGazeboSim):
        #     roll = self.state[s.ROLL]
        #     pitch = 0.0
        #     yaw = self.state[s.YAW]

        #     self.rotation_body2world = R.from_euler(
        #         'xyz',
        #         [roll, pitch, yaw]
        #     ).as_matrix().T

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
    

    def rk4_step(self, state, Fx, Mz, dt, dist):

        k1 = self.truck.dynamics(state, Fx, Mz, dist)
        k2 = self.truck.dynamics(state + 0.5*dt*k1, Fx, Mz, dist)
        k3 = self.truck.dynamics(state + 0.5*dt*k2, Fx, Mz, dist)
        k4 = self.truck.dynamics(state + dt*k3, Fx, Mz, dist)

        return state + dt/6.0 * (k1 + 2*k2 + 2*k3 + k4)



    def AtEnd(self):
        self.plotter.PlotAtEnd(self.state_data, self.observed_data, self.time_data)

 
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