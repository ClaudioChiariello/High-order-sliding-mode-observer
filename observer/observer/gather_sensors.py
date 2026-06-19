from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion

import numpy as np

"""A callback handler is not a ROS node. It doesn't need:

publishers
subscriptions
timers
parameters
node lifecycle

It just needs access to the node's state. So Inheritance would be a good solution since the class Callbacks would become a node"""
 

def angular_velocity_to_rpy_rates(roll, pitch, yaw_rate_body):
    """
    Convert body angular velocity [p,q,r] to roll/pitch/yaw derivatives.

    roll: phi
    pitch: theta
    yaw_rate_body: np.array([p,q,r])
    """

    phi = roll
    theta = pitch

    p, q, r = yaw_rate_body

    T = np.array([
        [1, np.sin(phi)*np.tan(theta),  np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi),              -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])

    return T @ np.array([p, q, r])

class Callbacks:

    def __init__(self, node):
        self.node = node
        self.imu_received = False
        self.odom_received = False
    
    def imu_callback(self, msg):
        
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # in radians
        self.roll, pitch, self.yaw = euler_from_quaternion(q)

        # Angular velocity
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        rpy_dot = angular_velocity_to_rpy_rates(
            self.roll,
            pitch,
            [wx,wy,wz]
        )

        self.dot_roll, _, self.dot_yaw = rpy_dot


        # Linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # self.node.get_logger().info(
        #     f"IMU acc: [{ax:.2f}, {ay:.2f}, {az:.2f}] "
        #     f"gyro: [{wx:.2f}, {wy:.2f}, {wz:.2f}]"
        # )
        self.imu_received = True
        self.update_error()


    def odom_callback(self, odom_msg):

        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

        self.vx = odom_msg.twist.twist.linear.x
        self.vy = odom_msg.twist.twist.linear.y

        # self.node.get_logger().info(
        #     f"Odom x={self.x:.2f} y={self.y:.2f} vx={self.vx:.2f}"
        # )

        self.odom_received = True
        self.update_error()


    def update_error(self):

        if self.imu_received and self.odom_received:
            
            self.node.state_x1 = np.array([
                self.x,
                self.y,
                self.roll,
                self.yaw
            ], dtype=np.float32)

            self.node.state_x2 = np.array([
                self.vx,
                self.vy,
                self.dot_roll,
                self.dot_yaw
            ], dtype=np.float32)

            self.node.estimated_error_x1 = self.node.state_x1 - self.node.observed_state_x1
            self.node.estimated_error_x2 = self.node.state_x2 - self.node.observed_state_x2

            self.odom_received = False
            self.imu_received = False
            self.node.triggered = True



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
