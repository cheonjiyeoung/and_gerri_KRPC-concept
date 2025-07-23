from pubsub import pub
import datetime
import time

import os, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import zlib
import base64
import threading
import time
import json
import array
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.examples.construction_operator import construction_command
from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()


class ConstructionBasePointcloudReceiver(Node):
    def __init__(self, robot_info, **params):
        super().__init__('publish_pointcloud_left')
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        pub.subscribe(self.receive_message, 'receive_message')
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/cloud_in_left', 1)
        self.receive_message_pointcloud = {}
        self.pub_message = PointCloud2()
        self.pub_message.height = 1
        self.pub_message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_message.is_bigendian = False
        self.pub_message.point_step = 20
        self.pub_message.is_dense = True
        self._lock = threading.Lock()
        self.point_array = None
        self.async_checking = False
        self.start_time = None
        self.origin_data = None
        self.init_time = None

    def connect(self):
        ros_pub_point_thread = threading.Thread(target=self.pub_point_cloud)
        ros_pub_point_thread.daemon = True
        ros_pub_point_thread.start()

    def pub_point_cloud(self):
        while True:
            start_time = time.time()
            if not self.receive_message_pointcloud:
                pass
            else:
                data = self.receive_message_pointcloud
                self.receive_message_pointcloud = None
                compressed_data = base64.b64decode(data)
                decompressed_data = zlib.decompress(compressed_data)
                point_array = array.array('B')
                point_array.frombytes(decompressed_data)
                self.pub_message.header = Header()
                self.pub_message.header.frame_id = 'left_camera_depth_optical_frame'
                self.pub_message.header.stamp = self.get_clock().now().to_msg()
                self.pub_message.width = int(len(point_array) / 20)
                self.pub_message.row_step = len(point_array)
                self.pub_message.data = point_array
                self.pointcloud_pub.publish(self.pub_message)
                end_time = time.time()
                print("size : ", self.pub_message.width, "time : ", end_time - start_time)
            time.sleep(0.001)

    def disconnect(self):
        self.sub_commander.disconnect()

    # def send_message(self, message):
    #     # pub.sendMessage('send_message', message=message)
    #     self.receive_message_pointcloud = message


    """
    Receives and handles messages from the robot.
    Specifically tracks robot status messages to calculate latency.
    """
    def receive_message(self, message):
        # print(message['compressed_data'], type(message['compressed_data']))
        self.receive_message_pointcloud = message['compressed_data']
