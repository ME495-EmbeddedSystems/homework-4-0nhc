#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import numpy as np
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


STOP = 0
NAVIGATING = 1

class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        self._initialized = False

        self.declare_parameter('front_distance_threshold', 6.0)
        self._front_distance_threshold = self.get_parameter(
            'front_distance_threshold').get_parameter_value().double_value

        self.declare_parameter('right_distance_threshold', 1.2)
        self._right_distance_threshold = self.get_parameter(
            'right_distance_threshold').get_parameter_value().double_value

        self.declare_parameter('left_distance_threshold', 1.2)
        self._left_distance_threshold = self.get_parameter(
            'left_distance_threshold').get_parameter_value().double_value

        self.declare_parameter('angular_z', 0.5)
        self._angular_z = self.get_parameter(
            'angular_z').get_parameter_value().double_value

        self.declare_parameter('linear_x', 0.8)
        self._linear_x = self.get_parameter(
            'linear_x').get_parameter_value().double_value

        self.declare_parameter('init_time', 3.0)
        self._init_time = self.get_parameter(
            'init_time').get_parameter_value().double_value

        markerQoS = QoSProfile(depth=10,
                               durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            LaserScan,
            '/scan',
            self._laser_callback,
            markerQoS)
        self._laser_data = None

        self._cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self._twist = Twist()

        self.create_timer(0.01, self._timer_callback)

    def _laser_callback(self,
                        msg: LaserScan):
        self._laser_data = msg

    def _initialize(self):
        self._twist.linear.x = 0.0
        self._twist.angular.z = self._angular_z
        self._cmd_publisher.publish(self._twist)
        time.sleep(self._init_time)
        self._twist.linear.x = 0.0
        self._twist.angular.z = 0.0
        self._cmd_publisher.publish(self._twist)
        self._initialized = True

    def _timer_callback(self):
        if not self._initialized:
            self._initialize()
        else:
            if (self._laser_data is not None):
                front_index = int((0 - self._laser_data.angle_min) /
                                  self._laser_data.angle_increment)
                right_index = int((-math.pi / 2 - self._laser_data.angle_min) /
                                  self._laser_data.angle_increment)
                left_index = int((math.pi / 2 - self._laser_data.angle_min) /
                                 self._laser_data.angle_increment)
                if 0 <= front_index < len(self._laser_data.ranges):
                    front_distance = self._laser_data.ranges[front_index]
                    right_distance = self._laser_data.ranges[right_index]
                    left_distance = self._laser_data.ranges[left_index]
                    # self.get_logger().info(f'Front distance: {front_distance})
                    # self.get_logger().info(f'Right distance: {right_distance}')
                    # self.get_logger().info(f'Left distance: {left_distance}')
                    if front_distance < self._front_distance_threshold:
                        self._twist.linear.x = 0.0
                        self._twist.angular.z = self._angular_z
                        self._cmd_publisher.publish(self._twist)
                    elif right_distance < self._right_distance_threshold:
                        self._twist.linear.x = self._linear_x
                        self._twist.angular.z = self._angular_z
                        self._cmd_publisher.publish(self._twist)
                    elif left_distance < self._left_distance_threshold:
                        self._twist.linear.x = self._linear_x
                        self._twist.angular.z = -self._angular_z
                        self._cmd_publisher.publish(self._twist)
                    else:
                        self._twist.linear.x = self._linear_x
                        self._twist.angular.z = -self._angular_z / 2
                        self._cmd_publisher.publish(self._twist)

                else:
                    self.get_logger().warn('Front index out of range!')
            else:
                self.get_logger().warn('No laser data!')


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    explore.destroy_node()
    rclpy.shutdown()
