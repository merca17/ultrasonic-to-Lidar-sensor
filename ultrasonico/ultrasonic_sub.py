#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi as mpi
from rclpy import qos

class UltrasonicSubNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_sub")
        self.ultrasonic_sub_ = self.create_subscription(LaserScan,"/scan",self.LaserScan_callback,qos_profile=qos.qos_profile_sensor_data)
    def LaserScan_callback(self,msg: LaserScan):
        self.get_logger().info(str(msg))
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSubNode()
    rclpy.spin(node)
    rclpy.shutdown()
