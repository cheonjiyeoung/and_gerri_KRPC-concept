from pubsub import pub
import datetime
import time

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.commander import manipulator_command
from gerri.operator.examples.construction_operator import construction_command
from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()


class ConstructionBaseCommander:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'sub_commander' in params:
            self.sub_commander = params['sub_commander']
        else:
            self.sub_commander = self.init_sub_commander(**params)

        if hasattr(self.sub_commander, 'init_base_commander'):
            self.sub_commander.init_base_commander(self)

        ### LEFT ARM #### 
        pub.subscribe(self.receive_message, 'receive_message')
        pub.subscribe(self.hello_universe, 'hello_message')
        pub.subscribe(self.left_stop_robot, 'left_stop_robot')
        pub.subscribe(self.left_moveJ, 'left_movej_message')
        pub.subscribe(self.left_moveL, 'left_movel_message')
        pub.subscribe(self.left_set_master_joint, 'left_set_master_joint')
        pub.subscribe(self.left_joint_ctrl_master, 'left_joint_ctrl_master')
        pub.subscribe(self.left_moving_gripper, 'left_moving_gripper')

        ### RIGHT ARM ####
        pub.subscribe(self.right_stop_robot, 'right_stop_robot')
        pub.subscribe(self.right_moveJ, 'right_movej_message')
        pub.subscribe(self.right_moveL, 'right_movel_message')
        pub.subscribe(self.right_set_master_joint, 'right_set_master_joint')
        pub.subscribe(self.right_joint_ctrl_master, 'right_joint_ctrl_master')
        pub.subscribe(self.right_moving_gripper, 'right_moving_gripper')

    def init_sub_commander(self, **params):
        if self.robot_model == 'doosanM1509':
            print("################################Go To Sub Control!!!###############################")
            from gerri.operator.examples.construction_operator.construction_sub_commander import ConstructionSubCommander
            return ConstructionSubCommander()

        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def connect(self):
        self.sub_commander.connect()

    def disconnect(self):
        self.sub_commander.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)


    """
    Receives and handles messages from the robot.
    Specifically tracks robot status messages to calculate latency.
    """
    def receive_message(self, message):
        # print("################### : ", messeage)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if topic == 'robot_status':
                ts = value['metadata'].get('timestamp')
                time_sync.record_latency(ts)
                avg, worst = time_sync.get_latency_stats()
                # print(f"📡 Ping: {avg:.2f} ms / 1% slow: {worst:.2f} ms")
                # print(value['robot_info']['joint_state']['position'])
                # print(value['robot_info']['pose']['position'])
                # print(value['robot_info']['pose']['orientation'])
                self.sub_commander.get_robot_state_left(value['robot_info']['left_robot']['state'], value['robot_info']['left_robot']['joint_state']['position'], value['robot_info']['left_robot']['pose']['position'], value['robot_info']['left_robot']['pose']['orientation'])
                self.sub_commander.get_robot_state_right(value['robot_info']['right_robot']['state'], value['robot_info']['right_robot']['joint_state']['position'], value['robot_info']['right_robot']['pose']['position'], value['robot_info']['right_robot']['pose']['orientation'])
            if 'target' in message:
                target = message['target']



    """
    Sends a "hello_universe" command message to all connected targets.
    """
    def hello_universe(self, hello_message):
        command = construction_command.hello_universe(message=hello_message, target='all')
        self.send_message(command)
    
    ####################    
    # LEFT ARM
    #####################
    def left_stop_robot(self, pause_type):
        print("Stop Robot", pause_type)
        command = construction_command.stop_robot(pause_type, target='left_arm')
        self.send_message(command)

    def left_moveJ(self, joint_angle, joint_velocity, joint_acceleration):
        command = construction_command.joint_ctrl(joint_angle, joint_velocity, joint_acceleration, target='left_arm')
        self.send_message(command)

    def left_moveL(self, point, angle, velocity, acceleration):
        command = construction_command.moveL(point, angle, velocity, acceleration, target='left_arm')
        self.send_message(command)

    def left_set_master_joint(self, master_start):
        command = construction_command.set_master_joint(master_start, target='left_arm')
        self.send_message(command)

    def left_joint_ctrl_master(self, joint_ctrl_master, joint_velocity, joint_acceleration, round):
        command = construction_command.joint_ctrl_master(joint_ctrl_master, joint_velocity, joint_acceleration, round, target='left_arm')
        self.send_message(command)

    def left_moving_gripper(self, moving_offset):
        command = construction_command.gripper_ctrl(moving_offset, target='left_arm')
        self.send_message(command)

    ####################    
    # RIGHT ARM
    #####################
    def right_stop_robot(self, pause_type):
        print("Stop Robot", pause_type)
        command = construction_command.stop_robot(pause_type, target='right_arm')
        self.send_message(command)

    def right_moveJ(self, joint_angle, joint_velocity, joint_acceleration):
        command = construction_command.joint_ctrl(joint_angle, joint_velocity, joint_acceleration, target='right_arm')
        self.send_message(command)

    def right_moveL(self, point, angle, velocity, acceleration):
        command = construction_command.moveL(point, angle, velocity, acceleration, target='right_arm')
        self.send_message(command)

    def right_set_master_joint(self, master_start):
        command = construction_command.set_master_joint(master_start, target='right_arm')
        self.send_message(command)

    def right_joint_ctrl_master(self, joint_ctrl_master, joint_velocity, joint_acceleration, round):
        print("Master Moving : ", joint_ctrl_master)
        command = construction_command.joint_ctrl_master(joint_ctrl_master, joint_velocity, joint_acceleration, round, target='right_arm')
        self.send_message(command)

    def right_moving_gripper(self, moving_offset):
        command = construction_command.gripper_ctrl(moving_offset, target='right_arm')
        self.send_message(command)
