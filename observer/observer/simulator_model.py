from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_matrix
from mujoco_ros2_control_msgs.msg import ContactState

from observer.utils.states import enum_obs_state, enum_outputs

import numpy as np

import threading

"""A callback handler is not a ROS node. It doesn't need:

publishers
subscriptions
timers
parameters
node lifecycle

It just needs access to the node's state. So Inheritance would be a good solution since the class Callbacks would become a node"""
 
 
class DynamicSimulatorCallback:

    def __init__(self, node):
        self.node = node

        num_states = 6
        num_outputs = 6 

        self.dynamic_simulator_state = np.zeros(num_states, dtype='float32')
        self.dynamic_simulator_output = np.zeros(num_outputs, dtype='float32')      
     
        self.vx = np.float32(0.0)
        self.vy = np.float32(0.0)
        self.wz = 0.0
        self.ay = np.float32(0.0) 
        self.phi_u = np.float32(0.0)

        self.previous_time = 0.0
        self.lock = threading.Lock()
    
    #Called every 20ms (50Hz) 
    def imu_callback(self, msg):
        
        #self.node.print_time()

        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, _, _ = euler_from_quaternion(q)
      
        with self.lock:

            self.dynamic_simulator_state[enum_obs_state.ROLL] = roll
            self.dynamic_simulator_output[enum_outputs.ROLL] = roll

            self.dynamic_simulator_state[enum_obs_state.WX] = msg.angular_velocity.x
            self.dynamic_simulator_output[enum_outputs.WX] = msg.angular_velocity.x

            self.dynamic_simulator_state[enum_obs_state.WZ] = msg.angular_velocity.z
            self.dynamic_simulator_output[enum_outputs.WZ] = msg.angular_velocity.z

            self.dynamic_simulator_output[enum_outputs.ACC_Y] = msg.linear_acceleration.y


    #Called every 50ms (20Hz) 
    def odom_callback(self, odom_msg):
    
        with self.lock:

            self.dynamic_simulator_state[enum_obs_state.VX] = odom_msg.twist.twist.linear.x
            self.dynamic_simulator_output[enum_outputs.VX] = -odom_msg.twist.twist.linear.x
            
            self.dynamic_simulator_state[enum_obs_state.VY] =  -odom_msg.twist.twist.linear.y
            
            

    def add_white_noise_gyro(self, signal):

        noise_density = 0.01 * np.pi/180 #rad/s/sqrt(Hz)
        imu_bandwidth = 415 #Hz
        standard_deviation = 2*noise_density * np.sqrt(imu_bandwidth)

        in_bias_stability = 10 *np.pi/180/3600   # rad/s
        
        
        return signal + np.random.normal(in_bias_stability, standard_deviation, np.shape(signal))


    def add_white_noise_acceleration(self, signal):

        noise_density = 60e-6  #g
        imu_bandwidth = 375 #Hz
        standard_deviation = 2*noise_density * np.sqrt(imu_bandwidth)

        in_bias_stability = 15e-6  #g
        
        
        return signal + np.random.normal(in_bias_stability, standard_deviation, np.shape(signal))


  
    def get_state_output(self):

        with self.lock:
            state = self.dynamic_simulator_state.copy()
            output = self.dynamic_simulator_output.copy()

        return state, output



    def contact_states_callback(self, msg: ContactState):

        # num_contacts = len(msg.ContactPair.)
        
        # if num_contacts == 0:
        #     return
            
        # 1. Extract all Z components into a clean list using a list comprehension
        z_forces = [c.force.z for c in msg.contacts]
        
        # Example: Print the total sum of Z forces (total vertical reaction force, e.g., weight/ground reaction)
        total_z_force = sum(z_forces)

        # 2. Iterate through each contact along with its corresponding body names and Z force
        # for b1, b2, force in zip(msg.contacts.body1_names, msg.contacts.body2_names, msg.contacts.force):
        #     self.get_logger().info(
        #         f"Contact: {b1} <-> {b2} | Force Z: {force.z:.2f} N"
        #     )
 
    def angular_velocity_to_rpy_rates(self, yaw_rate_body):
        """
        Convert body angular velocity [p,q,r] to roll/pitch/yaw derivatives.

        roll: phi
        pitch: theta
        yaw_rate_body: np.array([p,q,r])
        """

        phi = self.roll
        theta = self.pitch

        p, q, r = yaw_rate_body

        T = np.array([
            [1, np.sin(phi)*np.tan(theta),  np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi),              -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])

        return T @ np.array([p, q, r])

