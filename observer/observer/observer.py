#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_matrix

from .gazebo_model import GazeboCallback
from observer import robot
from observer.utils.data_plotter import DataPlotter

from observer.utils.states import enum_obs_state, enum_outputs

from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import GetParameters

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

class TruckStateObserver(Node):

    def __init__(self):
        super().__init__('truck_state_observer')

        self.parallel = ReentrantCallbackGroup()

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

        if self.get_parameter("use_sim_time").value:
            self.fixed_dt = 1e-3 #lo rendi abbastanza basso da usare il tempo di gazebo
            
            self.timer_gazebo = self.create_timer(
                self.fixed_dt, 
                self.GazeboControl,
                callback_group=self.parallel
            )

            self.gz_pub_ = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )

            self.joint_sub = self.create_subscription(
                JointState,
                '/joint_states',
                self.truck.joint_states_callback,
                qos_profile_sensor_data,
                callback_group=self.parallel
            )

            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data', #50Hz
                self.Gazebo.imu_callback,
                qos_profile_sensor_data,
                callback_group=self.parallel
            )

            self.odom_sub = self.create_subscription(
                Odometry,
                '/odometry', #20Hz
                self.Gazebo.odom_callback,
                qos_profile_sensor_data,
                callback_group=self.parallel
            )
        else:
            self.fixed_dt = 1e-2 #The observer should run at 10ms

        self.delta = np.zeros(2, dtype='float32')
        self.rotation_body2world = np.zeros((3,3), dtype='float32')
        
        num_states = 6
        num_outputs = 6

        # MODEL terms
        self.state = np.zeros(num_states, dtype=np.float64)

        self.output = np.zeros(num_outputs, dtype="float32")

        # OBSERVER terms
        self.observed_state = np.zeros(num_states, dtype=np.float64)

        self.observed_output = np.zeros(num_outputs, dtype="float32")

        # GAZEBO terms
        self.gazebo_state = np.zeros(num_states, dtype=np.float64)

        self.gazebo_output = np.zeros(num_outputs, dtype="float32")


        # Jacobian of the output vector field
        self.jacobian = np.zeros((num_outputs, num_states), dtype='float32')

        # PID Terms
        self.e_int = np.zeros(2,dtype="float32")
        self.previous_e = np.zeros(2,dtype="float32")

        # The controller frequency will always depend on this, and so the observer if it runs in the callback
        self.timer_observer = self.create_timer(
            self.fixed_dt, 
            self.ModelSimulator,
            #callback_group=self.group
        )

        self.counter = 0

        self.time_data = []
        self.output_data = []
        self.state_data = []
        self.observed_state_data = []
        self.observed_output_data = []
        self.gazebo_state_data = []
        self.gazebo_output_data = []
        self.sim_time = 0.0


        self.previous_time = None
        self.jacobian_reduced = False
        self.w = np.zeros(5, dtype=np.float64)


    def GazeboControl(self):
        msg = Twist()
        # Linear velocity (m/s)
        msg.linear.x = self.des_vel_x
        msg.linear.y = 0.0
        msg.angular.z = self.des_omega_z
        
        self.gz_pub_.publish(msg)


    def ModelSimulator(self):
        
        #self.print_time()

        #self.truck.computeJacobian()

        dt = self.fixed_dt
        u = self.PdController(dt, self.state)
        Fx, Mz = u

        dot_observed_state = None

        if self.get_parameter("use_sim_time").value:

            state, output = self.Gazebo.get_state_output()

            self.gazebo_state = state.copy()
            self.gazebo_output = output.copy()

            self.gazebo_state_data.append(state.copy())
            self.gazebo_output_data.append(output.copy())

        dot_x, J_x, h_meas, _ = self.truck.calculate_dynamics(self.state, Fx, Mz, False, dt+self.sim_time)
        
        self.state += dot_x*dt
        self.output = h_meas

        self.state_data.append(self.state.copy())
        self.output_data.append(self.output.copy())


        dot_observed_state, _, h_hat_meas, J_h = self.truck.calculate_dynamics(self.observed_state, Fx, Mz, False, dt+self.sim_time)

        #Remove the row of the Jacobian corresponding to h(x_6) = vx
        matrix_reduced = np.delete(J_h, enum_outputs.VX, axis=0)
        matrix_reduced = np.delete(matrix_reduced, enum_obs_state.VX, axis=1)
        self.jacobian = matrix_reduced
        self.observed_output = h_hat_meas

        self.JacobianObserver(dot_observed_state, dt)   
        
        self.observed_state_data.append(self.observed_state.copy())
        self.observed_output_data.append(self.observed_output.copy())

        self.sim_time += dt
        self.time_data.append(self.sim_time)


   
    def JacobianObserver(self, dot_observed_state, dt):
        
        J_inv = np.linalg.inv(self.jacobian)

        alpha = 50
        beta = 1.5

        # alpha = 0.0001
        # beta = 0.00005


        estimated_error = self.output - self.observed_output
        estimated_error = estimated_error[:5]

        #estimated_error[out.DOT_WX] = estimated_error[out.WX]

        root_abs_error = np.sqrt(np.abs(estimated_error))

        # 2. Overwrite just the specific indices using vectorized assignment
        root_abs_error[2] = np.abs(estimated_error[2]) ** (2/3)
        root_abs_error[3] = np.abs(estimated_error[2]) ** (1/3)

        sign_error = np.tanh(self.scale_tanh*estimated_error)

        self.w += beta*sign_error
        
        correction_term = alpha * root_abs_error * sign_error + self.w

        #Remove the 5th elements
        dot_observed_state_reduced = np.delete(dot_observed_state, enum_obs_state.VX)

        state_rate_reduced = dot_observed_state_reduced + J_inv @ correction_term
        # Re-insert a 0.0 value at index 4 so it matches the 6-element layout of self.observed_state

        state = np.insert(state_rate_reduced, 4, 0.0)

        self.observed_state += state * dt

        self.observed_state[4] = self.output[5]
            
        st = self.state
        obs = self.observed_state

        self.counter+=1
        if self.counter % (1/self.fixed_dt) == 0:
            self.get_logger().info(
            f"real (estimated)\n"
            f"Ang : r={st[enum_obs_state.ROLL]:7.3f} ({obs[enum_obs_state.ROLL]:7.3f})\n "
            f"Vel : vx={st[enum_obs_state.VX]:7.3f} ({obs[enum_obs_state.VX]:7.3f}) "
            f"vy={st[enum_obs_state.VY]:7.3f} ({obs[enum_obs_state.VY]:7.3f})\n"
            f"Rate: wx={st[enum_obs_state.WX]:7.3f} ({obs[enum_obs_state.WX]:7.3f})  "
            f"wz={st[enum_obs_state.WZ]:7.3f} ({obs[enum_obs_state.WZ]:7.3f})\n"
            f"dt={dt:.4f}"
        )

    def PdController(self, dt, current_state):

        Kp = np.array([100000, 2000000])
        Ki = np.array([100, 100.0])
        Kd = np.array([100, 10])

        vx_des, _, des_omega_z =  np.array([self.des_vel_x, 0, self.des_omega_z])
        
        control_error = np.array([
            vx_des - current_state[enum_obs_state.VX],
            des_omega_z - current_state[enum_obs_state.WZ]
        ])

        derivative_term = Kd*((control_error - self.previous_e)/dt)
        integral_term = Ki*self.e_int

        u = Kp*control_error + integral_term  
    
        self.previous_e = control_error
        self.e_int += control_error*dt

        return u
    

    def AtEnd(self):
        # Perché i vari thread potrebbero non restare sincronizzati
        n = min(len(self.time_data),
        len(self.state_data),
        len(self.output_data),
        len(self.observed_state_data),
        len(self.observed_output_data)
        )

        self.plotter.PlotAtEnd(self.state_data[:n],  self.output_data[:n],
                            self.observed_state_data[:n], self.observed_output_data[:n],
                            self.gazebo_state_data[:n], self.gazebo_output_data[:n],
                            self.time_data[:n])



    def print_time(self):

        now = self.get_clock().now()

        if self.previous_time is None:

            self.previous_time = now

            return
        
        dt = (now - self.previous_time).nanoseconds * 1e-9

        self.previous_time = now

        print(dt)


def main(args=None):

    rclpy.init(args=args)
    """Attenzione, perché con il MultiThreadExecutor i tempi delle chiamate delle callback per qualche motivo non sono più rispettati"""
    use_multiThread = False

    node = TruckStateObserver()

    try:
        if use_multiThread:
            executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.spin()  # Handled by the outer try-except now
        else:
            rclpy.spin(node) # Handled by the outer try-except now

    except KeyboardInterrupt:
        # This catches Ctrl+C cleanly for BOTH Gazebo and non-Gazebo modes!
        node.get_logger().info('Shutting down observer node cleanly...')
        
    finally:
        # Clean up the executor if it was initialized
        if use_multiThread and 'executor' in locals():
            executor.shutdown()
            
        # Run your custom end routine and destroy the node
        node.AtEnd()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()