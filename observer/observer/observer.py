#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from scipy.spatial.transform import Rotation

from .gather_sensors import Callbacks
from urdf_parser_py.urdf import URDF
from . import robot


"""dot_x1 = x2                      x = [x1] -> [x,y,roll,yaw]
    dot_x2 = f(x,u)                     [x2] -> [vx,vy,dot_roll,dot_yaw]"""
class TruckStateObserver(Node):

    def __init__(self):
        super().__init__('truck_state_observer')

        self.declare_parameter('scale_tanh', 10.0)
        self.scale_tanh = self.get_parameter('scale_tanh').value
        # self.x = np.float32(0.0)
        # self.y = np.float32(0.0)
        # self.roll = np.float32(0.0)
        # self.yaw = np.float32(0.0)

        self.state_x1 = np.zeros(4, dtype="float32")
        self.observed_state_x1 = np.zeros(4, dtype="float32")
        self.estimate_error_x1 = np.zeros(4, dtype="float32")

        self.state_x2 = np.zeros(4, dtype="float32")
        self.observed_state_x2= np.zeros(4, dtype="float32")
        self.estimate_error_x2 = np.zeros(4, dtype="float32")

        self.triggered = False

        
        self.cb = Callbacks(self)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.cb.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.cb.odom_callback,
            10
        )

        self.get_logger().info(
            "Truck state observer started"
        )   

        self.timer = self.create_timer(
            0.1,  # check every 100 ms
            self.HighOrderObserver
        )


    def HighOrderObserver(self):
        if not self.triggered:
            return
        
        self.triggered = False  # optional reset
        lambdaa = 100.0
        delta = np.sqrt(np.abs(self.estimate_error_x1))
        correction_term = lambdaa*delta*np.tanh(self.scale_tanh*self.estimate_error_x1)

        dot_x1_tilde = self.state_x2 + correction_term
        print(robot.Mass.shape)
        print("")
        #dot_x2_tilde = 



def main(args=None):

    rclpy.init(args=args)

    node = TruckStateObserver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()