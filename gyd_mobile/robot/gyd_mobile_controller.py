
import threading
import time
import os,sys
import requests
import asyncio
from pubsub import pub
from gyd_mobile.robot.utils.uart_conn import UART_Connector
from gyd_mobile.robot.utils.joystick_receiver import JoyStickReceiver

ROBOT_ADDR = "192.168.1.102"

class GydMobileController:
    def __init__(self):
        self.uart_conn = UART_Connector()
        self.move_start_event = asyncio.Event()
        self.move_done_event = asyncio.Event()
        self.patroll_running = False
        self.jsr = JoyStickReceiver(self)
        
    def connect(self):
        pass

    def disconnect(self):
        pass

    def dock_activate(self):
        command = "dock:start"
        self.uart_conn.uart_send(command)

    def data_spliter(self, data, keyword) -> list:
        escape_char = ['[', ']', '{', '}', ':']
        if keyword in data:
            idx = data.find(keyword) + len(keyword)
            value = data[idx:]
            for char in escape_char:
                value = value.replace(char, '')

            if ' ' in value:
                sp = value.split(' ')
            else:
                sp = value.split(',')
            
            # 빈 문자열 제거 (앞과 뒤 모두)
            sp = [item for item in sp if item != '']

            return sp
        return []

    def move(self, vx, vth):
        command = f"app_vel[{vx},{vth}]"
        self.uart_conn.uart_send(command)

    def move_coord(self,x,y,theta):
        rad = float(theta) / 180
        command = f"goal:nav[{x},{y},{rad}]"
        self.uart_conn.uart_send(command)

    def reloc_absolute(self,name):
        command = f"nav:reloc_absolute[{name}]"
        self.uart_conn.uart_send(command)

    def reloc(self,name):
        command = f"nav:reloc_name[{name}]"
        self.uart_conn.uart_send(command)

    def nav_cancel(self):
        command = "nav_cancel"
        self.uart_conn.uart_send(command)

    def nav_resume(self):
        command = "nav_resume"
        self.uart_conn.uart_send(command)

    def nav_pause(self):
        command = "nav_pause"
        self.uart_conn.uart_send(command)

    def set_poi(self,x,y,rad,type,name):
        command = f"set_flag_point[{x},{y},{rad},{type},{name}]"
        self.uart_conn.uart_send(command)

    def move_waypoint(self,poi):
        command = f"nav_point[{poi}]"
        self.uart_conn.uart_send(command)

    def send_message(self,message:dict):
        pub.sendMessage("solar_send",message=message)

    def get_path_plan(self):
        try:
            url = "http://192.168.1.102/reeman/global_plan"
            res = requests.get(url)
            current_path_plan = res.json().get("coordinates")
            return current_path_plan
        
        except Exception as e:
            print(f"err in get_path_plan . {e} (payload = {requests.get(url).json()})")

    def get_pose(self):
        url = f"http://{ROBOT_ADDR}/reeman/pose"
        result = requests.get(url).json()
        return result
    
    def get_mode(self):
        url = f"http://{ROBOT_ADDR}/reeman/get_mode"
        result = requests.get(url).json()
        return result

    def get_IMU(self):
        url = f"http://{ROBOT_ADDR}/reeman/imu"
        result = requests.get(url).json()
        return result
    
    def get_laser(self):
        url = f"http://{ROBOT_ADDR}/reeman/laser"
        result = requests.get(url).json()
        return result
    
    def get_power_management(self):
        url = f"http://{ROBOT_ADDR}/reeman/base_encode"
        result = requests.get(url).json()
        return result
    
    def get_speed(self):
        url = f"http://{ROBOT_ADDR}/reeman/speed"
        result = requests.get(url).json()
        return result 
    
    def get_nav_status(self):
        url = f"http://{ROBOT_ADDR}/reeman/nav_status"
        result = requests.get(url).json()
        return result

    def turn_on_power(self):
        url = f"http://{ROBOT_ADDR}/cmd/lock"
        response = requests.post(url, json={})
        return response.json()

    def turn_off_power(self):
        url = f"http://{ROBOT_ADDR}/cmd/unlock"
        response = requests.post(url, json={})
        return response.json()
    ### 수정 끝

if __name__ == "__main__":
    d = GydMobileController()
    while True:
        time.sleep(1)