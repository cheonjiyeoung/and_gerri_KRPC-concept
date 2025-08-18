from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.function.manipulator_function import ManipulatorFunction
from gerri.robot.function.mobile_function import MobileFunction
from gerri.robot.examples.spot.spot_sub_controller import SpotSubController
from gerri.robot.status_manager import StatusManager
import threading

class SpotBaseController(MobileFunction,ManipulatorFunction):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        self.mission_start_flag = False
        self.mission_start_running_flag = False
        self.mission_resume_flag = False
        self.mission_resume_running_flag = False

        self.sub_controller:SpotSubController = None
        pub.subscribe(self.receive_message,"receive_message")

    def receive_message(self, message):
        MobileFunction.receive_message(self,message=message)
        ManipulatorFunction.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message.get('value')
            option = message.get('option')
            if topic == '/joy': # rubberneck_virture_joystick
                """
                왼쪽 방향키 (좌우)= axes[0]
                왼쪽 방향키 (상하)= axes[1]

                오른쪽 조이스틱 (좌우)= axes[2]
                오른쪽 조이스틱 (상하)= axes[3]

                A = buttons[0]
                B = buttons[1]
                X = buttons[2]
                y = buttons[3]
                LB = buttons[4]
                RB = buttons[5]
                LT = buttons[6]
                RT = buttons[7]
                back = buttons[8]
                start = buttons[9]
                """
                print(f"\n\n joy {value}\n\n")
                axis = value.get("axes")
                buttons = value.get("buttons")
                axis_L_x = axis[1] # left x < 0
                axis_L_y = axis[0] # up y < 0
                axis_R_x = axis[2]
                axis_R_y = axis[3]
                A = buttons[0]
                B = buttons[1]
                X = buttons[2]
                Y = buttons[3]
                LB = buttons[4]
                RB = buttons[5]
                LT = buttons[6]
                RT = buttons[7]
                back = buttons[8]
                start = buttons[9]
                forword = buttons[12]
                backword = buttons[13]
                left = buttons[14]
                right = buttons[15]

                vx = 0 
                vy = 0
                vth = 0

                if forword > 0.7:
                    vx = 0.5
                elif backword > 0.7:
                    vx = -0.5
                if left > 0.7:
                    vy = 0.5
                elif right > 0.7:
                    vy = -0.5
                if axis_R_x < -0.7:
                    vth = 0.4
                elif axis_R_x > 0.7:
                    vth = -0.4
                if vx != 0 or vy != 0 or vth != 0:
                    self.mission_start_running_flag = False
                    value = {"vx":vx,"vy":vy,"vth":vth}
                    self.sub_controller.move(value=value)
                if A > 0.5:
                    self.sub_controller.stand()
                    self.sub_controller.walk()
                if B > 0.5:
                    self.sub_controller.sit()
                if X > 0.5:
                    self.sub_controller.dock()
                if Y > 0.5:
                    self.sub_controller.dock(value=True)
                if back > 0.5:
                    self.sub_controller.action_stop()
                if start > 0.5:
                    if LB > 0.5:
                        mission = "keti_2f-3f_round"
                        self.sub_controller.run_mission(mission)

                    elif RB > 0.5:
                        mission = "keti_3f-2f_round"
                        self.sub_controller.run_mission(mission)
                    else:
                        self.sub_controller.resume_mission()
                        self.mission_start_running_flag = True


                

                

            if topic == 'cmd_vel':
                # 'value': {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}}
                vx = value['linear']['x']
                vy = value['linear']['y']
                vth = value['angular']['z']
                # vx = vx * 0.2
                vth = vth * 0.2
                if vx != 0 or vth != 0:
                    value = {"vx":vx,"vy":vy, "vth":vth}
                    # self.sub_controller.move(value)
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'walk':
                            self.sub_controller.walk(value=value,option=option)
                        if topic == 'stand':
                            self.sub_controller.stand(value=value,option=option)
                        if topic == 'sit':
                            self.sub_controller.sit(value=value,option=option)
                        if topic == 'action_stop':
                            self.sub_controller.action_stop(value=value,option=option)
                        if topic == 'run_mission':
                            self.sub_controller.run_mission(value=value,option=option)
                        if topic == 'stop_mission':
                            self.sub_controller.stop_mission(value=value,option=option)
                        if topic == 'pause_mission':
                            self.sub_controller.pause_mission(value=value,option=option)
                        if topic == 'resume_mission':
                            self.sub_controller.resume_mission(value=value,option=option)
                        if topic == 'ptz_relative':
                            self.sub_controller.spot_ptz_relative(value=value,option=option)
                        if topic == 'ptz_absolute':
                            self.sub_controller.spot_ptz_absolute(value=value,option=option)
                        if topic == 'ptz_initialize':
                            self.sub_controller.spot_ptz_initialize(value=value,option=option)
                        if topic == 'spot_ptz_set_focus':
                            self.sub_controller.spot_ptz_set_focus(value=value,option=option)
                        if topic == 'spot_ptz_relative':
                            self.sub_controller.spot_ptz_set_focus_relative(value=value,option=option)
                        if topic == 'spot_ptz_set_focus_relative':
                            self.sub_controller.spot_ptz_set_focus_relative(value=value,option=option)
                        if topic == 'set_compositor':
                            self.sub_controller.set_compositor(value=value,option=option)
                        if topic == 'light_controll':
                            self.sub_controller.light_controll(value=value,option=option)
                        if topic == "predict":
                            self.sub_controller.predict_kospo(value=value,option=option)
                        if topic == "auto_return":
                            self.sub_controller.auto_return(value=value,option=option)
                        if topic == "get_lease":
                            self.sub_controller.get_lease(value=value,option=option)
                        if topic == "release_lease":
                            self.sub_controller.release_lease(value=value,option=option)
                        else:
                            self.send_message({'topic': 'callback_'+topic, 'value': 'callback_'+value, 'target': 'all'})
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()
