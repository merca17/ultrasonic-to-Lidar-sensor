#!usr/bin/env python3
# @autor: Juan Sebastian Mercado Allende
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi as mpi
from rclpy import qos
from gpiozero import DistanceSensor
import math as m
sensor = DistanceSensor(5,6, queue_len=10, max_distance= 5)
sensor1 = DistanceSensor(23,24, queue_len=10, max_distance= 5)
sensor2 = DistanceSensor(19,26, queue_len=10, max_distance= 5)



class UltrasonicMsgsNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_to_LaserScan")
        self.laser_scan_pub_ = self.create_publisher(LaserScan, "/scan",qos_profile=qos.qos_profile_sensor_data)
        self.timer_ = self.create_timer(0.5,self.send_laser_command)
        self.get_logger().info("Utrasonic to laser Msgs node has been started")
    def send_laser_command(self):
        msg = LaserScan()
        ## Configure msg header
        # timestamp in the header is the acquisition time of
        # the first ray in the scan.
        #
        # in frame_id, angles are measured around 
        # the positive Z axis (counterclockwise if z is up)
        # with zero angle being foward along the x axis

        msg.header.frame_id = 'laser_frame'
        msg.header.stamp = Node.get_clock(self).now().to_msg()

        msg.angle_min = -mpi/2                 # start angle of the scan [rad]
        msg.angle_max = mpi/2         # end angle of the scan [rad]
        msg.angle_increment = mpi/200    # angular distances between measurements [rad]

        msg.time_increment = 1.0/3        # time between measurements [seconds] if your scarnner is moving, this will be used in interpolating position of 3d points
        msg.scan_time = 1.0                 # time between scans

        msg.range_min = 0.05                 # minimum range value [m]
        msg.range_max = 3.0                 # maximum range value [m]

        for j in range(0,201):
            i=j*msg.angle_increment
            d1 = sensor.distance
            d2 = sensor1.distance
            d3 = sensor2.distance
            if(i<=mpi/3):
                msg._ranges.append(d1)
            elif(i>mpi/3 and i<=2*mpi/3):
                msg._ranges.append(d2)
            else:
                msg._ranges.append(d3)
            if(d1<0.1 or d2<0.1 or d3<0.1):
                msg.intensities.append(1000)
            else:
                msg.intensities.append(10)            
            
        # msg._ranges = [sensor.distance, sensor1.distance, sensor2.distance]                    # range data [m] (Note: values < range_min or > range_max should be descarted)
    
        #msg.intensities = []                # intensity data (if not provided leave the array empty)

        #publishing
        self.laser_scan_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicMsgsNode()
    rclpy.spin(node)
    rclpy.shutdown()