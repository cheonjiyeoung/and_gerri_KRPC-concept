import serial
import threading
import queue
import time
from pubsub import pub
import requests
import asyncio
import json

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.elsa.elsa_sub import ELSA_SUB
from _and_.elsa.elsa_pub import ELSA_PUB


map_list = {"keti_3F":{"name":"5a5389aa9f3507dd308d38e11bd38cc0","init_pose":"ev_out","floor":3},
            "keti_2F":{"name":"c9fb5ce7936f16519ce8e1743a8c2b0b","init_pose":"ev_out","floor":2}}

class GydMobileController:
    def __init__(self,port = "/dev/ttyUSB0", baudrate = 115200,timeout = 1):
        # Event loop
        self.robot_id = "gyd_mobile01"
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._start_loop, daemon=True).start()
        self.elevator_from_arrived_event = asyncio.Event()
        self.elevator_to_arrived_event = asyncio.Event()
        self.event_move_start = asyncio.Event()
        self.event_move_end = asyncio.Event()     

        # move floor buffer
        self.current_floor = None
        self.from_floor = None
        self.to_floor = None

        # ev
        self.elsapub = ELSA_PUB(elevator_id=1,robot_id=self.robot_id)
        elsasub = ELSA_SUB(elevator_id=1,robot_id=self.robot_id)
        th_elsa_receiver = threading.Thread(target=elsasub.loop_forever,daemon=True)
        th_elsa_receiver.start()
        pub.subscribe(self.ev_msg_handler,"ev_msg")

        ### status ###
        self.move_state = "IDLE"
        self.robot_name = "gyd_mobile_01"
        self.robot_type = {"category":"mobile"}
        self.robot_state = {"mode":"AUTO"}
        self.pose = {"2d" : {"x" : None, "y" : None, "th" : None}}
        
        self.battery = {
                               "percentage"             : None,
                               "power_supply_status"    : None}

        self.velocity = {"2d" : {"x" : 0.0, "th" : 0.0}}

        self.sensor = {"laser"      : None}
        
        self.map = {
            "frame_id"      : "KETI2F",
            "resolution"    : 0.05,
            "origin"        : {"x":-2.2361,"y":-14.3788},
            "size"          : {"width":790,"height":661}
        }
        self.current_floor = 2
        self.path_plan = {"global" : None}
        ###
        # UART Connection

        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        # UART Message Buffer
        self.buf = []
        self.q_data = queue.Queue(-1) 

        # Thread Objects
        self.th_handler = threading.Thread(target=self.uart_handler,daemon=True)
        self.th_receiver = threading.Thread(target=self.uart_receiver,daemon=True)
        # self.th_taskroutin = threading.Thread(target=self.task_routin,daemon=True)

        # UART Message Filter
        self.keywords_except = ["base_data","wheel_status",
                            "current_info","check_nodes","check_sensors"]
        
        self.keywords_status = ["base_vel", "nav_result", "core_data", "nav:pose",
                                "global_path", "battery_info","move_status","laser","ly_map","itpose"]

        self.status = 'connected'
        self.current_data = None
        self.th_handler.start()
        self.th_receiver.start()
        self.uart_send("nav:get_pose[on]") # respone pose actively
        self.uart_send("switch_lidar[on]") # respone laser actively

    def ev_msg_handler(self,msg):
        # msg = json.loads(msg)
        if "/floor" in msg["topic"]:
            self.current_floor = msg["value"]["floor"]
            print(f"current_floor = {self.current_floor}")
        if "/door" in msg["topic"]:
            if self.current_floor == self.from_floor:
                if msg["value"]["state"] == 0:
                    print("출발층 도착")
                    self.loop.call_soon_threadsafe(self.elevator_from_arrived_event.set)
            if self.current_floor == self.to_floor:
                if msg["value"]["state"] == 0:
                    print("목적층 도착")
                    self.loop.call_soon_threadsafe(self.elevator_to_arrived_event.set)

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
        
    def uart_send(self,res: str):
        bytes_data = res.encode('ascii')  # 문자열을 ASCII 바이트로 변환
        packet_length = len(bytes_data)   # 데이터 길이

        byte1 = bytearray(4 + packet_length)  # 전체 패킷 크기 (헤더 + 길이 + 데이터 + 체크섬)
        byte1[0] = 0xAA  # Frame Header 첫 번째 바이트
        byte1[1] = 0x54  # Frame Header 두 번째 바이트
        byte1[2] = packet_length  # 데이터 길이 (L)

        # 데이터 바이트 복사
        for i, b in enumerate(bytes_data):
            byte1[i + 3] = b  # 데이터 삽입

        # 체크섬 계산 (길이와 모든 데이터 XOR)
        checksum = packet_length
        for b in bytes_data:
            checksum ^= b
        
        byte1[-1] = checksum  # 마지막 바이트에 체크섬 저장
        packet = bytes(byte1)
        self.ser.write(packet) 

    def uart_handler(self):
        while True:
            # print(self.pose)
            # print(self.robot_state["mode"])
            data = self.q_data.get()
            # print(data)
            self.current_data = data
            for key in self.keywords_status: 
                if key in data:
                    sp = self.data_spliter(data,key)

                    if key == 'base_vel':   # wheel_state
                        self.velocity["2d"]["x"] = float(sp[0])
                        self.velocity["2d"]["th"] = float(sp[1])

                        pub.sendMessage("kesiroma",value = {"velo":{"vx":float(sp[0]),
                                                            "vth":float(sp[1])}})
                        
                    if key == 'nav_result':   # mission_allocation. sp[0] == 0 'idle' 
                        # print(data)
                        if sp[0] == "1":
                            # pub.sendMessage("robot_send",message={"state":"BUSY"})
                            pub.sendMessage("robot_send",message={"None"})
                            pub.sendMessage("kesiroma",value = {"status":"BUSY"})
                            self.loop.call_soon_threadsafe(self.event_move_start.set)
                        
                        if sp[0] == "3":
                            pub.sendMessage("robot_send",message={"DONE"})
                            pub.sendMessage("kesiroma",value = {"status":"IDLE"})
                            self.loop.call_soon_threadsafe(self.event_move_end.set)

                    if key == 'battery_info':   # battery
                        self.battery['voltage'] =       sp[1]
                        self.battery['temperature'] =   sp[2]
                        self.battery['capacity'] =      sp[4]


                    # if key == 'move_status': 
                    #     print(data)
                    if key == 'core_data':   # battery. sp[3] = percentage, sp[4] = charging status
                        now_charging = True if sp[4] == "2" else False
                        self.battery['percentage'] = sp[3]
                        self.battery['power_supply_status'] = now_charging
                        pub.sendMessage("kesiroma",value = {"battery":{"percent":sp[3],
                                                                       "now_charging":now_charging}})
                        
                    if key == 'nav:pose':   # pose
                        self.pose["2d"]['x'] = float(sp[0])
                        self.pose["2d"]['y'] = float(sp[1])
                        self.pose["2d"]['th'] = round(float(sp[2]),1)
                        msg = {"robot_id":"gyd_mobile01","pose":{"x":sp[0],"y":sp[1],"th":int(float(sp[2])*180)}}
                        pub.sendMessage("send_message", message=msg)
                        pub.sendMessage("kesiroma",value = {"pose":{"x":sp[0],"y":sp[1],"th":sp[2]}})
                    if key == 'global_path':   # mission_allowcation
                        self.path_plan['global'] = [float(path) for path in sp]

                    if key == 'laser':  # obstacle_detection
                        self.sensor["laser"] = float(sp[0])

                    if key == "ly_map":
                        pub.sendMessage("send_message", message=data)
                        print(data)

                    if key =="tpose":
                        pub.sendMessage("send_message", message=data)
                        print(data)
            time.sleep(0.03)
            # print(f"uart_q size : {self.q_data.qsize()}")

    def uart_receiver(self):
        signiture = []
        data = []
        try:
            while True:
                raw_data = self.ser.read()
                # 시작 헤더 발견
                if raw_data == b'\xaa':

                    if data:
                        self.q_data.put(''.join(data[1:-1]))
                        data = []

                    signiture = []
                    signiture.append(raw_data)

                elif raw_data == b'T':
                    signiture.append(raw_data)
                    data.append(raw_data.decode("utf-8"))

                elif signiture == [b'\xaa',b'T']:
                    try:
                        data.append(raw_data.decode("utf-8"))
                    except Exception as e:
                        # print(e)
                        pass

        except KeyboardInterrupt:
            self.ser.close()
            print("Serial connection closed.")

    def connect(self):
        self.status = 'connected'
    
    def disconnect(self):
        try:
            self.ser.close()
            self.status = 'disconnected'
            print("Disconnected from robot.")
        except:
            pass

    def change_mode(self,mode):
        print(f"Change_mode : mode = {mode}")
        if mode == 'AUTO':
            self.robot_state["mode"] = mode
            self.nav_resume()
            print(f"mode_change current = {mode}")


        if mode == 'TELEOP':
            self.robot_state["mode"] = mode
        print(f"Change_ctrlMode. current:{mode}")

    def joy(self, value):
        vx = value[0]
        vth = value[1]
        self.velocity["2d"]["x"]=float(vx)
        self.velocity["2d"]["th"]=float(vth)
        commend = f"app_vel[{vx},{vth}]"
        self.uart_send(commend)

    # def move_waypoint(self,poi):
    #     asyncio.create_task(self._move_waypoint(poi))

    def move_waypoint(self, waypoint):
        asyncio.create_task(self._move_waypoint(waypoint))

    async def _move_waypoint(self,poi):
        if self.move_state == "IDLE":
            self.loop = asyncio.get_running_loop()
            commend = f"nav_point[{poi}]"
            print(commend)
            self.uart_send(commend)
            await self.event_move_start.wait()
            self.event_move_start.clear()
            self.move_state = "BUSY"
            print(f"{poi}이동 시작")
            await self.event_move_end.wait()
            self.event_move_end.clear()
            print(f"{poi}이동 완료")
            self.move_state = "IDLE"

    def move_floor(self, from_to):
        asyncio.create_task(self._move_floor(from_to))


    async def _move_floor(self, from_to):
        origin = from_to['from']
        destination = from_to['to']

        self.from_floor = origin
        self.to_floor = destination
        await self.move_waypoint(f"{origin+1}F_E1_W")
        print("ev 호출.")
        self.elsapub.ev_call(origin,destination)
        await self.elevator_from_arrived_event.wait()
        self.elevator_from_arrived_event.clear()
        print("ev 탑승 대기 알림.")
        self.elsapub.ev_in_wait()
        await self.move_waypoint(f"{origin+1}F_E1")
        self.elsapub.ev_in()
        self.reloc_absolute(f"{destination+1}F_E1")
        print("ev 탑승 완료 알림.")
        await self.elevator_to_arrived_event.wait()
        self.elevator_to_arrived_event.clear()
        print("ev 하차 대기 알림.")
        self.elsapub.ev_out_wait()
        await self.move_waypoint(f"{destination+1}F_E1_E")
        print("하차 완료")
        print("ev 하차 완료 알림.")
        self.elsapub.ev_out()
        self.state = "IDLE"
        self.from_floor = None
        self.to_floor = None            

    def move_coord(self, coord):
        x = coord[0]
        y = coord[1]
        theta = coord[2]
        rad = float(theta) / 180
        commend = f"goal:nav[{x},{y},{rad}]"
        self.uart_send(commend)
    
    def dock(self,name):
        self.move_waypoint(name)

    def get_BatteryInfo(self):
        commend = "get_battery_info"
        self.uart_send(commend)

    def powerOFF(self):
        commend = "power_off"
        self.uart_send(commend)

    def get_CurrentMap(self):
        commend = "nav:current_map"
        self.uart_send(commend)

    def relocate(self,value):
        pose = value
        commend = f"nav:reloc_absolute[{pose}]"
        self.uart_send(commend)

    def reloc_point(self,poi:str):
        commend = f"nav:reloc_name[{poi}]"
        self.uart_send(commend)

    def nav_cancel(self):
        commend = "nav_cancel"
        self.uart_send(commend)

    def nav_resume(self):
        commend = "nav_resume"
        self.uart_send(commend)        

    def set_vel(self,vel:float):
        commend = f"max_vel[{vel}]"
        self.uart_send(commend)

    def set_footprint(self,width,length): # (m)
        commend = f"footprint[{width},{length}]"
        self.uart_send(commend)

    def active_StatusNav(self):
        commend = "update_dynamic"
        self.uart_send(commend)

    def set_poi(self,x,y,rad,type,name):
        commend = f"set_flag_point[{x},{y},{rad},{type},{name}]"
        self.uart_send(commend)

    def stop(self):
        self.uart_send("move[0,0]")
        print("Robot stopped.")

    def map_change(self, map):
        name = map_list[map]["name"]
        floor = map_list[map]["name"]
        self.current_floor = floor
        self.uart_send(f"call_web[apply_map:{name}]")
        if map_list[map]["name"] == "keti_3F":
            self.map = {
            "frame_id"      : "keti_3F",
            "resolution"    : 0.05,
            "origin"        : {"x":-12.9257,"y":-7.26509},
            "size"          : {"width":782,"height":383}
        }
        if map_list[map]["name"] == "keti_3F":
            self.map = {
            "frame_id"      : "keti_2F",
            "resolution"    : 0.05,
            "origin"        : {"x":-2.2361,"y":-14.3788},
            "size"          : {"width":790,"height":661}
        }

    def _start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

# Example usage
if __name__ == "__main__":
    test = GydMobileController("/dev/ttyUSB0",115200,1) 
    import os,sys
    CURRENT_FILE = os.path.abspath(__file__)
    VENV_DIR = os.path.dirname(sys.executable) 
    PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
    # cmd="request:cpu_performance"
    # test.uart_send(cmd)
    sys.path.insert(0, PROJECT_ROOT)
    # test.reloc_absolute("2F_chargingpile")
    asyncio.run(test.move_floor(f=2,t=1))
    # test.map_change(map=map_list["keti_3F"]["name"],floor=2,pose=map_list["keti_3F"]["init_pose"])

    # while True:
    #     time.sleep(1)
    #     pass



