#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class CMDVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher_node")

        self._cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._yaw_publisher = self.create_publisher(String, 'yaw', 10)    


        self._odometry_subscriber = self.create_subscription(Odometry, 'odom', self.odometryCallback, 10)
        
        
        self._cmd_vel_message = Twist()
        self._odom_message = Odometry()
        self._yaw_message = String()

        self._cmd_vel_message.angular.z = math.pi/6

        self._total_time = 0.0

        self._start = self.get_clock().now().to_msg().sec
        self._end = self.get_clock().now().to_msg().sec

        self._dt = 0.01

        self._timer = self.create_timer(self._dt, self.mainLoop)

    
    def debug(self, message):
        self.get_logger().info(str(message))


    def odometryCallback(self, message):
        self._odom_message = message
        roll, pitch, yaw = self.euler_from_quaternion(self._odom_message.pose.pose.orientation.x, self._odom_message.pose.pose.orientation.y, self._odom_message.pose.pose.orientation.z, self._odom_message.pose.pose.orientation.w)
        print("Odom orientation x: " + str(self._odom_message.pose.pose.orientation.x))
        print("Odom orientation y: " + str(self._odom_message.pose.pose.orientation.y))
        print("Odom orientation z: " + str(self._odom_message.pose.pose.orientation.z))
        print("Odom orientation w: " + str(self._odom_message.pose.pose.orientation.w))
        print("\n")

        self._yaw_message.data = str(yaw)
        self._yaw_publisher.publish(self._yaw_message)


    def mainLoop(self):
        #if self._total_time < 6:
        if True:
            self._start = self.get_clock().now().to_msg().sec
            #self.debug("cmd_vel_publisher: " + str(self._total_time))

            #self._cmd_vel_publisher.publish(self._cmd_vel_message)
            self._total_time = self._total_time + (self._start - self._end)
            # self._total_time = self._total_time + self._dt
            self._end = self._start

            # self.debug("cmd_vel_publisher: " + str(self._total_time))

        else:
            self._cmd_vel_message.angular.z = 0.0
            self._cmd_vel_publisher.publish(self._cmd_vel_message)
            self._timer.reset()


 
    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CMDVelPublisher()

    rclpy.spin(cmd_vel_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
        