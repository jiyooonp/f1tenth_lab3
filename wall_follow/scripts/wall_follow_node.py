#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """

    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        self.pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)
        self.scan_callback = self.create_subscription(
            LaserScan, lidarscan_topic, self.scan_callback, 10)

        # TODO: set PID gains
        self.kp = 0.7
        self.kd = 0.75
        self.ki = 0.1

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.prev_time = 0.0

        # TODO: store any necessary values you think you'll need
        self.speed = 0.5
        self.L = 0.5

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        # TODO: implement

        # angles = np.linspace(range_data.angle_min, range_data.angle_max, len(range_data.ranges))

        # Find the index of the angle closest to the given angle
        # index = np.argmin(np.abs(angles - angle))
        index = np.floor((angle - range_data.angle_min) / range_data.angle_increment).astype(int)

        # Return the range at the given index
        return range_data.ranges[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        # TODO:implement

        # theta
        theta = np.pi / 3

        angle_a = np.pi/2 - theta
        angle_b = np.pi / 2

        # a 
        a = self.get_range(range_data, angle_a)

        # b
        b = self.get_range(range_data, angle_b)


        # alpha 
        self.alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))

        # D_t
        delta_t = b*np.cos(self.alpha)

        print("Delta_t: ", delta_t)


        delta_t_1 = delta_t + self.L * np.sin(self.alpha)

        # error
        self.prev_error = self.error

        self.error = dist - delta_t_1

        return -self.error


    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        self.integral += error * 1e-9

        # TODO: Use kp, ki & kd to implement a PID controller

        # self.u_t = steering angle
        time_diff = (self.curr_time - self.prev_time) * 1e-9
        self.u_t = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)/time_diff

        self.prev_time = self.curr_time

        # speed
        if abs(self.u_t) <= np.pi/18:
            self.speed = 1.5
        elif abs(self.u_t) <= np.pi/9:
            self.speed = 1.0
        else:
            self.speed = 0.5
        

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = self.u_t
        drive_msg.drive.speed = self.speed #* 2.0    

        # print speed and steering angle
        # print("Speed: ", self.speed)
        # print("Steering Angle: ", self.u_t)
        self.pub.publish(drive_msg)


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.curr_time = msg.header.stamp.nanosec
        error = self.get_error(msg, 1.0)
        # TODO: replace with error calculated by get_error()
        # TODO: calculate desired car velocity based on error
        self.pid_control(error)  # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
