import sys
import os


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from gerri.robot.status_manager import StatusManager
from gerri.robot.raas_dataset_builder import RldsDatasetBuilder

from gerri.robot.robot_tools.gyd_mobile.config_reeman_robot import CAMERA_INFO

class MobileController:
    def __init__(self, robot_name, robot_model, **params):
        self.robot_name = robot_name
        self.robot_category = 'mobile'
        self.robot_model = robot_model

        self.controller = self._initialize_robot(robot_model, **params)
        self.status_manager = StatusManager(self.robot_name, self.robot_category, self.robot_model, self.controller)
        self.dataset_builder = RldsDatasetBuilder(self.controller, CAMERA_INFO, interval=0.1,
                                              save_dir="/media/dev2kng/SanDisk/RLDS_dataset")

    def _initialize_robot(self, robot_model, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """
        if robot_model == 'gyd_mobile':
            from gyd_mobile.reeman_controller import ReemanMobileController
            return ReemanMobileController(robot_ip=params['ip'])
        else:
            raise ValueError(f"Unsupported robot model: {robot_model}")

    def message_handler(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_name or message['target'] == 'all':
                    if topic == 'move_waypoint':
                        self.controller.move_waypoint(value)
                    elif topic == 'move_coord':
                        self.controller.move_coord(value)
                    elif topic == 'dock':
                        self.controller.dock(value)
                    elif topic == 'joy':
                        self.controller.joy(value)


    def connect(self):
        self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()
