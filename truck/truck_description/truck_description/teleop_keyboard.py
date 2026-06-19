#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.linear_speed = 0.5      # m/s
        self.angular_speed = 1.0     # rad/s

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.key = None

        self.get_logger().info(
            'Use WASD to move, X to stop, Q to quit'
        )

    def get_key(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                "No terminal attached. Keyboard teleop requires a TTY."
            )
            return

        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.0
        )

        key = None

        if rlist:
            key = sys.stdin.read(1)

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key

    def update(self):

        key = self.get_key()

        msg = Twist()

        if key == 'w':
            msg.linear.x = self.linear_speed

        elif key == 's':
            msg.linear.x = -self.linear_speed

        elif key == 'a':
            msg.angular.z = self.angular_speed

        elif key == 'd':
            msg.angular.z = -self.angular_speed

        elif key == 'x':
            pass

        elif key == 'q':
            self.get_logger().info('Exiting...')
            rclpy.shutdown()
            return

        else:
            return

        self.pub.publish(msg)


def main():

    rclpy.init()

    node = KeyboardTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error(
            "No terminal attached. Keyboard teleop requires a TTY."
        )
        return
    node.settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            node.settings
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
