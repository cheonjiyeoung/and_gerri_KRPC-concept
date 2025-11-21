import threading
import json
import uuid
import time
from pubsub import pub
from datetime import datetime
from and_gerri.gerri.utils.time_sync_manager import TimeSyncManager


TimeSync = TimeSyncManager(sample_count=1)

class RaasDatasetBuilder:
    def __init__(self, controller, camera_info, interval=0.1, discount_factor=1, save_dir="/home/keti/RaaS_dataset", **kwargs):
        self.controller = controller.status
        self.camera_info = camera_info
        self.interval = interval
        self.status = vars(self.controller)
        self.step_count = 0
        self.now_recording = False
        self.discount_factor = discount_factor
        self.save_dir = save_dir
        self.episode_id = None
        self.episode_name = None
        self.episode_path = None
        self.is_last = False
        self.is_terminal = False

        self.robot_info = self.get_robot_status()
        self.last_status = self.robot_info
        self.start_episode_timestamp = datetime.fromisoformat(str(TimeSync.get_scaled_server_time())).strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # ✅ PubSub 구독
        pub.subscribe(self.start_episode, 'start_episode')
        pub.subscribe(self.end_episode, 'end_episode')
        pub.subscribe(self.fail_episode, 'fail_episode')

        # ✅ 폴더가 없으면 생성
        os.makedirs(self.save_dir, exist_ok=True)


    def get_robot_status(self):
        if hasattr(self.controller, 'update_status'):
            self.controller.update_status()
        robot_status = vars(self.controller)
        robot_info = {
            'robot_id': self.controller.robot_id,
            'robot_type': self.controller.robot_type,
        }
        # MOBILE
        if 'robot_state' in robot_status:
            robot_info['robot_state'] = robot_status['robot_state']
        if 'pose' in robot_status:
            robot_info['pose'] = robot_status['pose']
        if 'battery_state' in robot_status:
            robot_info['battery_state'] = robot_status['battery_state']
        if 'robot_shape' in robot_status:
            robot_info['robot_shape'] = robot_status['robot_shape']
        if 'wheel_state' in robot_status:
            robot_info['wheel_state'] = robot_status['wheel_state']
        if 'velocity' in robot_status:
            robot_info['velocity'] = robot_status['velocity']
        if 'sensor' in robot_status:
            robot_info['sensor'] = robot_status['sensor']
        if 'map' in robot_status:
            robot_info['map'] = robot_status['map']
        if 'path_plan' in robot_status:
            robot_info['path_plan'] = robot_status['path_plan']

        # MANIPULATOR
        if 'joint_state' in robot_status:
            robot_info['joint_state'] = robot_status['joint_state']

        self.last_status = robot_info
        return robot_info


    def save_episode_info(self, end_episode=False):
        """📡 에피소드 정보를 JSON 파일로 저장"""
        info = {
            'robot_info': {
                'robot_name': self.controller.robot_id,
                'robot_type': self.controller.robot_type,
            },
            'camera_info': self.camera_info,
        }

        if end_episode:
            episode_info = {
                'episode_id': self.episode_id,
                'start_timestamp': self.start_episode_timestamp,
                'end_timestamp': datetime.fromisoformat(str(TimeSync.get_scaled_server_time())).strftime("%Y%m%d_%H%M%S_%f")[:-3],
                'time_period_seconds': self.interval,
                'total_steps': self.step_count,
            }

        else:
            episode_info = {
                'episode_id': self.episode_id,
                'start_timestamp': self.start_episode_timestamp,
                'end_timestamp': None,
                'time_period_seconds': self.interval,
                'total_steps': None,
            }

        info['episode_info'] = episode_info


        # ✅ JSON 파일 저장
        info_file = os.path.join(self.episode_path, f"{self.episode_id}_info.json")
        with open(info_file, "w") as f:
            json.dump(info, f, indent=4)

        print(f"✅ 에피소드 정보 저장 완료: {info_file}")


    def setup_episode_folder(self):
        """📁 RLDS 데이터셋 폴더 생성"""
        self.episode_path = os.path.join(self.save_dir, self.episode_name, self.episode_id)
        os.makedirs(self.episode_path, exist_ok=True)  # ✅ 폴더가 없으면 생성

        print(f"📁 RLDS 에피소드 폴더 생성: {self.episode_path}")

    def reward_function(self):
        if 'robot_state' in self.last_status:
            if self.last_status['robot_state']:
                if self.last_status['robot_state']["mode"]:
                    if self.last_status['robot_state']["mode"] in ['teleop', 'TELEOP']:
                        reward = 2
                    elif self.last_status['robot_state']["mode"] in ['auto', 'AUTO']:
                        reward = 1
                    else:
                        reward = 0
                    return reward
        else:
            return 0

    def action_function(self):
        if 'robot_type' in self.last_status:
            if self.last_status['robot_type']['category'] == 'mobile':
                action = self.last_status['velocity']['2d']
                return action
        else:
            return None

    def save_episode_step(self):
        """📡 현재 Step 정보를 JSON 파일로 저장"""
        step_data = {
            'step': self.step_count,
            'is_first': self.step_count == 0,
            'is_last': self.is_last,
            'is_terminal': self.is_terminal,
            'observation': self.get_robot_status(),
            'action': self.action_function(),
            'reward': self.reward_function(),
            'discount': self.discount_factor,
            'metadata': {
                'timestamp': datetime.fromisoformat(str(TimeSync.get_scaled_server_time())).strftime("%Y%m%d_%H%M%S_%f")[:-3],
                'uuid': str(uuid.uuid4()),
            },
        }

        pub.sendMessage('save_frame', file_path=self.episode_path, file_name=self.step_count)

        # ✅ 카메라 이미지 경로 설정
        image_info = {}
        for cam_name in self.camera_info.keys():
            filename = f"{cam_name}_{self.step_count:05d}.jpg"
            file_path = os.path.join(self.episode_path, filename)
            image_info[cam_name] = {"file_path": file_path}

        # ✅ 관측 정보에 image 경로 포함
        step_data["observation"]["images"] = image_info

        # ✅ JSON 파일 저장
        step_file = os.path.join(self.episode_path, f"step_{self.step_count:05d}.json")
        with open(step_file, "w") as f:
            json.dump(step_data, f, indent=4)

        # print(f"✅ Step {self.step_count} 저장 완료: {step_file}")



    def start_episode(self, episode_name):
        """🎬 새로운 에피소드 시작"""
        if not self.now_recording:
            print('start episode', episode_name)
            self.now_recording = True
            self.start_episode_timestamp = datetime.fromisoformat(str(TimeSync.get_scaled_server_time())).strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self.episode_name = episode_name
            self.episode_id = f"{episode_name}_{self.robot_info['robot_id']}_{self.start_episode_timestamp}"
            self.setup_episode_folder()  # ✅ 폴더 생성
            self.step_count = 0
            self.save_episode_info()

            def loop():
                while self.now_recording:
                    self.save_episode_step()
                    self.step_count += 1
                    time.sleep(self.interval)

            thread = threading.Thread(target=loop, daemon=True)
            thread.start()

    def end_episode(self):
        """🏁 에피소드 종료 및 데이터 저장"""
        if self.now_recording:
            self.now_recording = False
            self.is_last = True
            self.is_terminal = True
            self.save_episode_step()
            self.save_episode_info(end_episode=True)


    def fail_episode(self, fail_steps=600):
        """🚨 실패 발생 시, 지난 600개 스텝의 Reward를 0으로 변경"""
        if not self.episode_path:
            print("🚨 [ERROR] 에피소드 폴더가 없습니다! 실패 처리를 건너뜁니다.")
            return

        print(f"🚨 [FAIL] 에피소드 실패! 마지막 600개 Step의 보상을 0으로 변경합니다.")

        # ✅ 에피소드 폴더에서 Step JSON 파일 리스트 가져오기
        step_files = sorted([
            f for f in os.listdir(self.episode_path) if f.startswith("step_") and f.endswith(".json")
        ])

        # ✅ 최근 600개 스텝만 선택 (파일이 600개 미만이면 전부 선택)
        last_600_steps = step_files[-fail_steps:] if len(step_files) > fail_steps else step_files

        for step_file in last_600_steps:
            step_path = os.path.join(self.episode_path, step_file)

            # ✅ Step JSON 파일 읽기
            with open(step_path, "r") as f:
                step_data = json.load(f)

            # ✅ Reward 값을 0으로 변경
            step_data["reward"] = 0

            # ✅ 변경된 데이터 다시 저장
            with open(step_path, "w") as f:
                json.dump(step_data, f, indent=4)

            print(f"✅ [FAIL 처리] {step_file} → reward = 0")

        print("🏁 [FAIL 처리 완료] 지난 600개 스텝의 보상이 0으로 변경되었습니다.")



if __name__ == "__main__":

    CURRENT_FILE = os.path.abspath(__file__)
    VENV_DIR = os.path.dirname(sys.executable)
    PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
    sys.path.insert(0, PROJECT_ROOT)

    from avatar_darm.robot.robot_tools.reeman.config_reeman_robot import *


    class DummyController:
        def __init__(self):
            """가짜 컨트롤러 (테스트용)"""
            self.robot_name = "TestBot"
            self.robot_type = {'category': 'mobile',
                          'model': 'gyd'}
            self.robot_state = {'mode': 'auto',}
            self.pose = [0.0, 0.0, 0.0]
            self.battery = 100
            self.robot_shape = "quadruped"
            self.wheel_state = "stopped"
            self.joint_state = [0.0] * 6
            self.velocity = {'2d':[0.0] * 6}


    test_robot = DummyController()
    # ✅ RLDS 빌더 인스턴스 생성
    dataset_builder = RldsDatasetBuilder(
        controller=test_robot,
        camera_info=CAMERA_INFO,
        interval=0.1,
    )


    from avatar_darm.robot.network_tools.webrtc.cam_manager import CameraManager
    cam1 = CameraManager(camera_name='front_cam', camera_index=2, width=640, height=480, fps=30)
    cam2 = CameraManager(camera_name='rear_cam', camera_index=0, width=640, height=480, fps=30)
    cam1.start()
    cam2.start()

    time.sleep(3)
    # ✅ 에피소드 시작
    pub.sendMessage("start_episode", episode_name="test_episode")

    time.sleep(10)  # 10초 동안 데이터 수집

    pub.sendMessage("fail_episode")
    time.sleep(5)

    test_robot.robot_state = {'mode': 'teleop', }

    time.sleep(10)  # 10초 동안 데이터 수집


    # ✅ 에피소드 종료
    pub.sendMessage("end_episode")

    # ✅ 저장된 데이터 확인
    episode_path = dataset_builder.episode_path
    print(f"\n📁 저장된 에피소드 폴더: {episode_path}")
    print(f"📂 저장된 파일 목록: {os.listdir(episode_path)}")