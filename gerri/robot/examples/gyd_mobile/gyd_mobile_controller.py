
import threading
import time
import os,sys
import requests
import asyncio
from pubsub import pub

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.gyd_mobile.robot_status import RobotStatus
from gerri.robot.examples.gyd_mobile.utils.uart_conn import UART_Connector
ROBOT_ADDR = "192.168.1.102"
reason_map = {
    0: "nomal",
    1: "docking...",
    2: "EMS",
    3: "charging",
    4: "no route",
    5: "docking failed",
    6: "abnomal pose",
    7: "too far",
    8: "no fixed route",
    9: "faild to read point"
}
class GydMobileController:
    def __init__(self,robot_status=None):
        self.uart_conn = UART_Connector()
        # self.q_data = self.uart_conn.q_data
        self.status:RobotStatus = robot_status
        self.move_start_event = asyncio.Event()
        self.move_done_event = asyncio.Event()
        self.patroll_running = False
        
    def connect(self):
        th_mainloop = threading.Thread(target=self.main_loop,daemon=True)
        # th_mainloop.start()

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
    
    # # 메인루프
    # def main_loop(self):
    #     while True:
    #         # data = self.q_data.get()
    #         # try:
    #         #     if "battery_info" in data:
    #         #         sp = self.data_spliter(data,"battery_info")
    #         #         self.status.battery_state['voltage'] = sp[1]
    #         #         self.status.battery_state['temperature'] = sp[2]
    #         #         self.status.battery_state['capacity'] = sp[4]
    #         # except Exception as e:
    #         #     print(f"err in battery_info : {e}")

    #         # try:
    #         #     if "special_area" in data:
    #         #         sp = self.data_spliter(data,"special_area")
    #         #         # if sp[0] == '3F_office_out':
    #         # except Exception as e:
    #         #     print(f"err in special_area : {e}")

    #         self._get_nav_status()
    #         time.sleep(0.3)

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

    def patroll_async(self,route):
        loop = asyncio.get_event_loop()
        loop.create_task(self.patrolling(route))

    def move_waypoint_async(self,poi,solar_callback=False,uuid=None):
        loop = asyncio.get_event_loop()
        loop.create_task(self._move_waypoint_async(poi,solar_callback,uuid))

    async def _move_waypoint_async(self,poi,solar_callback=False,uuid=None): 
        self.move_waypoint(poi)
        self.move_start_event.clear()
        await self.move_start_event.wait()
        print(f"move to {poi} start !")
        self.move_done_event.clear()
        await self.move_done_event.wait()
        print(f"move to {poi} done !")
        if solar_callback:
            message = {"topic":"solar_callback","uuid":uuid}
            print(f"solar_send : {message}")
            pub.sendMessage(topicName="solar_send",message=message)
        return True

    def move_waypoint(self,poi):
        command = f"nav_point[{poi}]"
        self.uart_conn.uart_send(command)

    def send_message(self,message:dict):
        pub.sendMessage("solar_send",message=message)

    def logging_with_send(self,log:str):
        print(log)
        message = {"robot_message":{"robot_id":self.robot_id,"message":log}}
        self.send_message(message)


    def _get_path_plan(self):
        try:
            url = "http://192.168.1.102/reeman/global_plan"
            res = requests.get(url)
            current_path_plan = res.json().get("coordinates")
            return current_path_plan
        
        except Exception as e:
            print(f"err in get_path_plan . {e} (payload = {requests.get(url).json()})")

    def _get_pose(self):
        url = f"http://{ROBOT_ADDR}/reeman/pose"
        result = requests.get(url).json()
        x = result.get("x",0)
        y = result.get("y",0)
        th = result.get("theta",0)
        return x,y,th
    
    def _get_mode(self):
        url = f"http://{ROBOT_ADDR}/reeman/get_mode"
        result = requests.get(url).json()
        mode = result.get("mode",0)
        if mode == 1:
            current_mode = "mapping"
        elif mode == 2:
            current_mode = "nav"
        else:
            current_mode = "ERROR"
        return current_mode

    def _get_IMU(self):
        url = f"http://{ROBOT_ADDR}/reeman/imu"
        result = requests.get(url).json()
        a = result.get("a",0)
        g = result.get("g",0)
        return a,g
    
    def _get_laser(self):
        url = f"http://{ROBOT_ADDR}/reeman/laser"
        result = requests.get(url).json()
        coordinates = result.get("coordinates")
        return coordinates
    
    def _get_power_management(self):
        url = f"http://{ROBOT_ADDR}/reeman/base_encode"
        result = requests.get(url).json()
        battery_percentage = result.get("battery",None)
        charge_flag = result.get("chargeFlag",None)
        ems = result.get("emergencyButton",None)
        now_charging = True if charge_flag == 2 else False
        emergency_stop = True if ems == 0 else False
        return battery_percentage, now_charging, emergency_stop
    
    def _get_speed(self):
        url = f"http://{ROBOT_ADDR}/reeman/speed"
        result = requests.get(url).json()
        vx = result.get("vx",0)   
        vth = result.get("vth",0)   
        return vx, vth
    
    def _get_nav_status(self):
        url = f"http://{ROBOT_ADDR}/reeman/nav_status"
        result = requests.get(url).json()
        res = result.get("res")   
        reason = result.get("reason")   
        goal = result.get("goal",0)   
        dist = result.get("dist",0)   
        mileage = result.get("mileage",0)   

        info = None
        flag = None
        nav_state = {}

        if res == 1:
            flag = "START"
            self.move_start_event.set()
        elif res == 2:
            flag = "PAUSED"
        elif res == 3:
            flag = "SUCCEED" if reason == 0 else "FALIED"
            self.move_done_event.set()
        elif res == 4:
            flag = "CANCELED"
            self.move_done_event.set()
        info = f"{reason_map.get(reason)}"
        nav_state = {"goal":goal,"result":flag,"info":info}

        return nav_state, dist, mileage
    ### 수정 시작
    def turn_on_power(self):
        url = f"http://{ROBOT_ADDR}/cmd/lock"
        response = requests.post(url, json={})
        return response.json()

    def turn_off_power(self):
        url = f"http://{ROBOT_ADDR}/cmd/unlock"
        response = requests.post(url, json={})
        return response.json()
    ### 수정 끝

    async def patrolling(self,route:list):
        if not self.patroll_running:
            self.patroll_running = True
            for poi in route:
                print(f"({poi}) 주행시작")
                await self._move_waypoint_async(poi)
                print(f"({poi}) 주행완료")
            self.patroll_running = False
        else:
            print("patroll is running")
            

if __name__ == "__main__":
    d = GydMobileController()
    d.main_loop()