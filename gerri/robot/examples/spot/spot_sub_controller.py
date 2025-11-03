import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.spot.spot_mission_schaduler import MissionSchaduler
from gerri.robot.examples.spot.robot_status import RobotStatus
import threading
import time
import datetime
from pubsub import pub
import requests
import json
import base64

class SpotSubController:
    def __init__(self,spot_info):
        self.spot_info = spot_info
        if spot_info["using_ptz"]:   
            from gerri.robot.examples.spot.spot_ptz import SpotPTZ
            self.robot_controller = SpotPTZ(spot_info)
        elif spot_info["using_arm"]:
            from gerri.robot.examples.spot.spot_arm import SpotArm
            self.robot_controller = SpotArm(spot_info)
        else:
            from gerri.robot.examples.spot.spot_controller import SpotController
            self.robot_controller = SpotController(spot_info)
        
        self.last_light_level = [0,0,0,0]
        self.status = None
        self._lock = threading.Lock()
        self.last_fault_id = None

    def connect(self):
        self.status = RobotStatus(robot_id=self.base_controller.robot_id,
                                    model=self.base_controller.robot_model,
                                    category=self.base_controller.robot_category)
        self.th_update_status = threading.Thread(target=self._update_loop,daemon=True)
        self.th_update_status.start()
        self.robot_controller.connect()

        # # 비동기 이벤트 루프를 새 스레드에서 실행하도록 설정
        # loop = asyncio.new_event_loop()
        # threading.Thread(target=self._run_spot_callback_handler, args=(loop,), daemon=True).start()
        # self.schaduler = MissionSchaduler(self,loop)
        # self.schaduler.exec_scheduler_cron(mission='2F3F_ROUND',t_value=35)

    def disconnect(self):
        self.th_update_status.join()
        self.robot_controller.lease_keepalive.shutdown()
        
    def _update_loop(self):
        while True:
            try:
                self._lock.acquire()
                self.robot_controller._robot_state_task.update()
                
                _, fiducial_objects = self.robot_controller.spot_camera.get_fiducial_objects()
                mission_state = self.robot_controller.spot_autowalk.get_mission_state()

                self.status.common_status_parser(self.robot_controller.robot_state)
                self.status.fiducial_info_parser(fiducial_objects)
                self.status.mission_status_parser(mission_state)
                try:
                    if mission_state.questions is not None:
                        question = mission_state.questions[0]  # 첫 번째 질문을 선택
                        question_id = question.id
                        question_text = question.text
                        options = {}

                        for option in question.options:  # 수정된 부분
                            options[option.text] = option.answer_code  # 수정된 부분

                        # 메시지 포맷 정의
                        topic = "spot_question"
                        value = {
                            "id": question_id,
                            "text": question_text,
                            "options": options
                        }
                        message = {
                            "topic": topic,
                            "value": value
                        }

                        self.base_controller.send_message(message)

                except Exception as e:
                    # print(f"General error: {e}")
                    pass
                try:
                    fault_state = self.robot_controller.robot_state.behavior_fault_state.faults
                    if fault_state:  # 리스트가 비어있지 않으면
                        for fault in fault_state:
                            # 각 fault 객체에 대한 처리
                            last_fault_id = fault.behavior_fault_id
                            if self.last_fault_id is None or self.last_fault_id != last_fault_id:
                                self.last_fault_id = last_fault_id
                                cause = fault.cause
                                # cause는 enum 값이므로 string 비교가 아닌 enum으로 비교
                                if cause == fault.Cause.CAUSE_FALL:
                                    pass
                                else:
                                    self.auto_return()
                except Exception as e:
                    print(f"Error in behavior fault handling: {e}")

                if self.spot_info["using_ptz"]:
                    self.get_ptz_status()
                self._lock.release()
                time.sleep(0.2)
                if self.status.spot_mission_state == 3:
                    pub.sendMessage("end_episode")
                if self.status.spot_mission_state == 1:
                    pub.sendMessage("fail_episode")
            except Exception as e:
                pass
    # def answer_from_user(self,value=None,option=None):

    def get_lease(self,value=None,option=None):
        self.robot_controller.initialize()

    def release_lease(self,value=None,option=None):
        try:
            self.robot_controller.spot_act.safe_power_off()
            self.robot_controller.spot_act._estop_keepalive.shutdown()
            self.robot_controller.lease_keepalive.shutdown()
        except:
            print("lease shutdown aleady")


    def get_ptz_status(self):
        ptz_focus = self.robot_controller.spot_ptz_get_focus()
        ptz_pose = self.robot_controller.spot_ptz_get_position()
        self.status.ptz_pose_parser(ptz_pose)
        self.status.ptz_focus_parser(ptz_focus)

    def answer_question(self,value=None,option=None):
        pass

    def walk(self,value=None,option=None):
        if self.status.mode == 'Stand':
            self.robot_controller.spot_act.walk()
            self.status.mode = 'Walk'

    def stand(self,value=None,option=None):
        self.robot_controller.spot_act.stand()
        self.status.mode = 'Stand'

    def sit(self,value=None,option=None):
        self.robot_controller.spot_act.sit()
        self.status.mode = 'Sit'

    def action_stop(self,value=None,option=None):
        self.robot_controller.spot_act.action_stop()

    def move(self,value=None,option=None):
        if self.status.mode=='Walk':
            self.status.controll_mode = "teleop"
            vx = value.get('vx',0)
            vy = value.get('vy',0)
            vth = value.get('vth',0)
            self.robot_controller.spot_act.move(vx=vx,vy=vy,vth=vth)

    def run_mission(self,value=None,option=None):
        self.status.controll_mode = "auto"
        pub.sendMessage("start_episode",episode_name=f"mission_{value}")
        mission_name = value
        self.robot_controller.spot_autowalk.handle_mission(mission_name)

    def stop_mission(self,value=None,option=None):
        self.robot_controller.spot_autowalk.stop_mission()

    def pause_mission(self,value=None,option=None):
        self.status.controll_mode = "teleop"
        self.robot_controller.spot_autowalk.pause_mission()

    def resume_mission(self,value=None,option=None):
        if self.status.controll_mode == 'teleop':
            self.status.controll_mode = "auto"
            self.robot_controller.spot_autowalk.restart_mission()

    def dock(self,value=None,option=None):
        undock = value
        if undock:
            self.status.mode = "Stand"
        self.robot_controller.spot_act.dock(undock)

    # SPOT PTZ
    def spot_ptz_absolute(self,value=None,option=None):
        pan = int(value.get("pan", 0))
        tilt = int(value.get("tilt", 0))
        zoom = int(value.get("zoom", 1))
        ptz_position = self.robot_controller.spot_ptz_absolute(
                                                 pan,
                                                 tilt,
                                                 zoom)
        return ptz_position
    
    def spot_ptz_relative(self,value=None,option=None):
        pan_step = value.get("pan", 0)
        tilt_step = value.get("tilt", 0)
        zoom_step = value.get("zoom", 1) 
        ptz_position = self.robot_controller.spot_ptz_absolute(
                                                 pan_step,
                                                 tilt_step,
                                                 zoom_step)
        return ptz_position

    def spot_ptz_initialize(self,value=None,option=None):
        self.robot_controller.spot_ptz_initialize()

    def spot_ptz_set_focus(self,value=None,option=None):
        mode = value.get('mode')
        if mode == 'manual':
            dist = value.get('dist')
        else:
            dist = None
        self.robot_controller.spot_ptz_set_focus(mode,dist)

    def spot_ptz_set_focus_relative(self,value=None,option=None):
        step = value
        current_dist = self.status.ptz_focus.get('dist')
        if current_dist is not None:
            target_dist = current_dist+step
            self.robot_controller.spot_ptz_set_focus_relative(target_dist)

    def set_compositor(self,value=None,option=None):
        name = value
        self.robot_controller.spot_set_compositor(name)

    def light_controll(self,value=None,option=None):
        print(f"subcontroller ok . value : {value}")
        if value == "on":
            level = [1,1,1,1]
        else:
            level = [0,0,0,0]
        self.last_light_level = level
        self.robot_controller.spot_light(level)

    def predict_kospo(self,value=None,option=None):
        self.robot_controller.spot_light(self.last_light_level)
        target = value
        timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        filename = f"{target}_{timestamp}.jpg"
        pub.sendMessage("acuire_frame",file_name=filename)
        raw_file_dir = "/home/keti/KOSPO_RAW/"
        self.spot_ptz_absolute(value={"pan":145,"tilt":0,"zoom":1})
        for i in range(5):
            try:
                raw_file = raw_file_dir + filename
                ret_dict = self.call_predict_api(value=raw_file)
                if ret_dict is not None:
                    return ret_dict

            except Exception as e:
                print(f"failed to load file (err={e}). file name = {raw_file}")
                time.sleep(1)
        
        
    def call_predict_api(self,value=None,option=None):
        try:
            img_path = value
            with open(img_path, 'rb') as f:    
                r = f.read()                     
            img_b64 = base64.encodebytes(r)
            img_b64 = img_b64.decode('ascii')

            query_list = {
                'img_b64': img_b64,
                'point_num': 1
            }
            json_str = json.dumps(query_list, sort_keys=False, indent=4)

            url = 'http://kmsqwet.iptime.org:7800/img_proc'
            headers = {"Content-type": "application/json"}
            ret = requests.post(url=url, data=json_str, headers=headers) 
            print(f"[http results] \nret = {ret}")
            ret_byte = ret.text.encode('utf-8')
            ret_str = ret_byte.decode('utf-8')
            ret_dict = json.loads(ret_str)
            print(ret_dict)
            return ret_dict
        except Exception as e:
            print(f"err in call predict api : {e}")

    def auto_return(self,value=None,option=None):
        self.robot_controller.auto_return_force()
            

    # SPOT_ARM



