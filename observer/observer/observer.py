#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_matrix


from observer.model import dynamic_model 
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

        self.declare_parameter("des_omega_z", 0.4) #giving a desired angular vel prevent the velocity to reach a steady state
        self.des_omega_z = self.get_parameter('des_omega_z').value
        
        # Friend Function 
        self.truck = dynamic_model.DynamicModel()
        
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

        # Differentiator state
        self.omega_x_obs = np.float32(0.0)
        self.dot_omega_x_obs = np.float32(0.0) 

        self.zita2 = np.float32(0.0)
        self.zita3 = np.float32(0.0)

        self.dot_omega_x_obs_prev = np.float32(0.0) 

        self.phi_s_data = []


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

        if self.get_parameter("use_sim_time").value:

            state, output = self.Gazebo.get_state_output()

            self.gazebo_state = state.copy()
            self.gazebo_output = output.copy()

            self.gazebo_state_data.append(state.copy())
            self.gazebo_output_data.append(output.copy())
        

        # Model Dynamic
        u = self.truck.PdController(self.state, self.des_vel_x, self.des_omega_z, dt)
        Fx, Mz = u

        dot_x, J_x, h_meas, _ = self.truck.calculate_real_dynamics(self.state, Fx, Mz, True, dt+self.sim_time)
        
        self.state += dot_x*dt
       
        self.output = h_meas
        self.output[[enum_outputs.WX, enum_outputs.WZ]] = self.Gazebo.add_white_noise_gyro(h_meas[[enum_outputs.WX, enum_outputs.WZ]])
        self.output[[enum_outputs.ACC_Y]] = self.Gazebo.add_white_noise_acceleration(h_meas[enum_outputs.ACC_Y])
        

        self.state_data.append(self.state.copy())
        self.output_data.append(self.output.copy())


        # Observer's Dynamic
        u = self.truck.PdController(self.observed_state, self.des_vel_x, self.des_omega_z, dt)
        Fx, Mz = u

        dot_observed_state, _, h_hat_meas, J_h = self.truck.calculate_obs_dynamics(self.observed_state, Fx, Mz)

        #Remove the row of the Jacobian corresponding to h(x_6) = vx
        matrix_reduced = np.delete(J_h, enum_outputs.VX, axis=0)
        matrix_reduced = np.delete(matrix_reduced, enum_obs_state.VX, axis=1)
        self.jacobian = matrix_reduced
        # Compute Jacobian correction
        self.observed_output = h_hat_meas
        self.JacobianObserver(dot_observed_state, self.output.copy(), dt)   

        self.observed_state_data.append(self.observed_state.copy())
        self.observed_output_data.append(self.observed_output.copy())


        self.sim_time += dt
        self.time_data.append(self.sim_time)


    def JacobianObserver(self, dot_observed_state, real_output, dt):
        
        J_inv = np.linalg.inv(self.jacobian)

        alpha = 50
        beta = 1.5

        estimated_error = real_output - self.observed_output
        estimated_error = estimated_error[:5]

        root_abs_error = np.sqrt(np.abs(estimated_error))

        # 2. Overwrite just the specific indices using vectorized assignment
        root_abs_error[enum_outputs.DOT_WX] = np.abs(estimated_error[enum_outputs.WX]) ** (1/3)

        sign_error = np.tanh(self.scale_tanh*estimated_error)

        self.w += beta*sign_error
        self.w[[enum_outputs.DOT_WX]] = 0.0
    
        correction_term = alpha * root_abs_error * sign_error + self.w

        #Remove the 5th elements
        dot_observed_state_reduced = np.delete(dot_observed_state, enum_obs_state.VX)

        state_rate_reduced = dot_observed_state_reduced + J_inv @ correction_term
        # Re-insert a 0.0 value at index 4 so it matches the 6-element layout of self.observed_state

        state = np.insert(state_rate_reduced, 4, 0.0)
        
        self.observed_state += state * dt
        self.observed_state[enum_obs_state.VX] = real_output[enum_outputs.VX]
        
        self.STDifferentiator(real_output[enum_outputs.WX], dt)

        self.observed_output[enum_outputs.DOT_WX] = self.dot_omega_x_obs

        phi_s = self.observed_state[enum_obs_state.ROLL] - self.observed_state[enum_obs_state.PHI_U]
        self.phi_s_data.append(phi_s)
        self.print()



    """To copute the transportation term I correctly need an estimation of dot_omega"""
    def STDifferentiator(self, y, dt):
        

        error = self.omega_x_obs - y

        lambda1 = 25
        lambda2 = 100 

        # lambda1 = 1.5*np.sqrt(L)
        # lambda2 = 1.1*L

        # smooth sign (replace with np.sign if chattering is acceptable)
        s = np.tanh(self.scale_tanh * error)

        # observer equations
        dot_z1 =  - 10 * np.sqrt(np.abs(error)) * s + self.zita2
        zita2_dot = -5 * s + self.zita3
        self.omega_x_obs += dot_z1 * dt
        self.zita2 += zita2_dot * dt

        dot_z2 = -lambda1 * np.sqrt( np.abs( self.dot_omega_x_obs - self.zita2)) *np.tanh(self.scale_tanh * (self.dot_omega_x_obs - self.zita2)) + self.zita3
        zita3_dot = -lambda2 * np.tanh(self.scale_tanh * (self.dot_omega_x_obs - self.zita2))
        self.dot_omega_x_obs += dot_z2*dt
        self.zita3 += zita3_dot*dt
        # integrate




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
                            self.time_data[:n], self.phi_s_data[:n])



    def print(self):

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
            # f"dot_WX={real_output[enum_outputs.DOT_WX]:7.3f} ({self.dot_omega_x_obs:7.3f})\n"
            # f"differentiator_wx={real_output[enum_outputs.WX]:7.3f} ({self.omega_x_obs:7.3f})\n"
        )


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