from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_matrix

from observer.utils.states import obs_state as s
from observer.utils.states import outputs as out

import numpy as np

import threading

"""A callback handler is not a ROS node. It doesn't need:

publishers
subscriptions
timers
parameters
node lifecycle

It just needs access to the node's state. So Inheritance would be a good solution since the class Callbacks would become a node"""
 
 
class GazeboCallback:

    def __init__(self, node):
        self.node = node
        self.imu_received = False
        self.odom_received = False
        num_states = 6
        num_outputs = 6 
        self.gazebo_state = np.zeros(num_states, dtype='float32')
        self.gazebo_output = np.zeros(num_outputs, dtype='float32')
        
     
        self.vx = np.float32(0.0)
        self.vy = np.float32(0.0)
        self.wz = 0.0
        self.ay = np.float32(0.0) 
        self.phi_u = np.float32(0.0)

        self.previous_time = 0.0
        self.lock = threading.Lock()
    
    #Called every 20ms (50Hz) 
    def imu_callback(self, msg):
        
        now = self.node.get_clock().now().nanoseconds * 1e-9

        if self.previous_time == 0.0:
            self.previous_time = now
            return

        dt = now - self.previous_time
        self.previous_time = now

        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, _, _ = euler_from_quaternion(q)

        if(self.node.UseGazeboSim):
            self.node.rotation_body2world = quaternion_matrix(q)[:3, :3].T
      
        with self.lock:

            self.gazebo_state[s.ROLL] = roll
            self.gazebo_output[out.ROLL] = roll

            self.gazebo_state[s.WX] = msg.angular_velocity.x
            self.gazebo_output[out.WX] = msg.angular_velocity.x

            self.gazebo_state[s.WZ] = msg.angular_velocity.z
            self.gazebo_output[out.WZ] = msg.angular_velocity.z

            self.gazebo_output[out.ACC_Y] = msg.linear_acceleration.y

        self.imu_received = True

    #Called every 50ms (20Hz) 
    def odom_callback(self, odom_msg):

        with self.lock:

            self.gazebo_state[s.VX] = odom_msg.twist.twist.linear.x

            self.gazebo_output[out.VX] = odom_msg.twist.twist.linear.x

            self.gazebo_state[s.VY] =  odom_msg.twist.twist.linear.y

        self.odom_received = True


    def get_state_output(self):

        with self.lock:
            state = self.gazebo_state.copy()
            output = self.gazebo_output.copy()

        return state, output


 
 
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


# def tf_callback(self, msg):

#     for t in msg.transforms:

#         if t.child_frame_id == "base_link":

#             x = t.transform.translation.x
#             y = t.transform.translation.y

#             self.get_logger().info(
#                 f"TF base_link: {x:.2f}, {y:.2f}"
#             )


# def joint_callback(self, msg):

#     # Example:
#     # left_rot_joint_a1 velocity
#     if "left_rot_joint_a1" in msg.name:

#         idx = msg.name.index("left_rot_joint_a1")

#         velocity = msg.velocity[idx]
#         position = msg.position[idx]

#         self.get_logger().info(
#             f"Wheel velocity: {velocity:.3f} rad/s "
#             f"position: {position:.3f}"
#         )


#     # steering
#     if "left_steer_joint_a1" in msg.name:

#         idx = msg.name.index("left_steer_joint_a1")

#         steering_angle = msg.position[idx]

#         self.get_logger().info(
#             f"Steering angle: {steering_angle:.3f} rad"
#         )
