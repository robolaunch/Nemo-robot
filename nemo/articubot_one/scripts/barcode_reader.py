#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf2_ros
import json
import time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
import numpy as np
import math

class BarcodeReader(Node):
    def __init__(self):
        super().__init__('barcode_reader')
        self.tf_pose = TFMessage()
        self.robot_pose = PoseStamped()

        self.pose = PoseStamped()

        self._message_to_send = String()

        # Subscribers
        self.barcode_subscription = self.create_subscription(String, 'barcode', self.barcode_callback, 10)
        self.tf_subscription      = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        self.pose_subscription    = self.create_subscription(PoseStamped, 'robot_position', self.robot_pose_callback, 10)

        # Publishers
        self._barcode_publisher = self.create_publisher(String, 'all_barcodes', 10)

        self.barcode_list = []

        self._barcode_messages = {
            "barcodes": []
        }


    def robot_pose_callback(self, data):
        self.pose = data



    def barcode_callback(self, msg):
        barcode_message = json.loads(msg.data)
        scanner_id = str(barcode_message["scanner_id"])
        barcode = str(barcode_message["barcode"])

        if not barcode in self.barcode_list:
            self.barcode_list.append(barcode)

            print("barcode: " + msg.data)
            print("barcode: " + barcode + " barcode list : " + str( self.barcode_list))
            print()

            roll, pitch, yaw = self.quaternion_to_euler([self.pose.pose.orientation.w,
                                                  self.pose.pose.orientation.x,
                                                  self.pose.pose.orientation.y,
                                                  self.pose.pose.orientation.z,])
            
            magnitude = 0.5
            direction_vec = [math.cos(yaw+math.pi*0.5)*magnitude, math.sin(yaw+math.pi*0.5)*magnitude]

            json_obj = {
                "robotId": "1",
                "fleetId": "1",
                "sensorId": str(scanner_id),
                "barcode": str(barcode),
                "waypoint":
                {
                    "x": self.pose.pose.position.x + direction_vec[0],
                    "y": self.pose.pose.position.y + direction_vec[1],
                    "z": 0.0,
                    "yaw" : yaw* 180/math.pi
                }
            }

            self._barcode_messages["barcodes"].append(json_obj)

            #msg_send = String()
            #print(self._barcode_messages)
            #msg_send.data = json.dumps(self._barcode_messages, ensure_ascii=False)
            self._message_to_send.data = json.dumps(self._barcode_messages, ensure_ascii=False)
        print(self._barcode_messages)

        self._barcode_publisher.publish(self._message_to_send)



    def tf_callback(self, data):
        self.tf_pose = data
        map2odom_pose = data.transforms[0].transform.translation
        odom2map_pose = data.transforms[0].transform.translation

        self.robot_pose.pose.position.x = odom2map_pose.x + map2odom_pose.x
        self.robot_pose.pose.position.y = odom2map_pose.y + map2odom_pose.y
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()



    def quaternion_to_euler(self, quaternion):
        w, x, y, z = quaternion

        # Calculate Euler angles (roll, pitch, yaw)
        roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))
        yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        return roll, pitch, yaw


        
if __name__ == '__main__':
    rclpy.init()
    node = BarcodeReader()
    rclpy.spin(node)
    rclpy.shutdown()