import asyncio
from pubsub import pub

import os, sys
import rclpy
from rclpy.node import Node
import os
import sys
import time

from rclpy.logging import get_logger
from pubsub.pub import sendMessage
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
import threading
import struct
import zlib
import zstandard as zstd
import base64

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class ConstructionBasePointCloud(Node):
    def __init__(self, robot_info, **params):
        super().__init__('pointcloud_subscriber_right')
        self.subscription_pointcloud = self.create_subscription(PointCloud2, '/camera2/camera2/depth/color/points',
                                                                self.point_callback, 1)
        # self.subscription_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
        #                                                    self.depth_callback, 10)
        self.point_message = {}
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        self.sleep_time = 0.1
        pub.subscribe(self.receive_message,"receive_message")

    def point_callback(self, msg):
        self.point_message = msg

    def sending_thread(self):
        while True:
            self.send_pointcloud()
            time.sleep(self.sleep_time)

    def send_pointcloud(self):
        if not self.point_message:
            pass
        else:
            size = int(len(self.point_message.data) /20)
            self.sleep_time = size * (1/50000)
            # if size > 50000:
            #     self.sleep_time = 1
            # elif size <= 50000 and size >= 40000:
            #     self.sleep_time = 0.1
            # elif size < 40000:
            #     self.sleep_time = 0.05
            start_time = time.time()

            msg_bytes = self.point_message.data.tobytes()
            self.point_message = None
            compressed_data = zlib.compress(msg_bytes, level=1)
            base64_encoded_data = base64.b64encode(compressed_data).decode('utf-8')

            json_message = {"compressed_data": base64_encoded_data}
            self.send_message(json_message)
            end_time = time.time()
            # self.number += 1
            print("time : ", end_time - start_time, " number: ", size)

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        print(message)
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        print("Start Get PointCloud")
        self.thread = threading.Thread(target=self._ros_loop, daemon=True)
        self.thread.start()
        self.send_thread = threading.Thread(target=self.sending_thread, daemon=True)
        self.send_thread.start()

    def _ros_loop(self):
        rclpy.spin(self)

    def disconnect(self):
        rclpy.shutdown()
