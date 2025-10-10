import os, sys
import time
import asyncio
import threading

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.gyd_mobile.gyd_robot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO
from gerri.robot.examples.gyd_mobile.utils.solar_robot_client import SolarClient
from gerri.robot.raas_dataset_builder import RaasDatasetBuilder
# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video_info=VIDEO_INFO,
    audio_info=AUDIO_INFO,
)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.gyd_mobile.gyd_base_controller import GydBaseController
from gerri.robot.examples.gyd_mobile.gyd_sub_controller import GydSubController

gyd_base_controller = GydBaseController(ROBOT_INFO)
gyd_sub_controller = GydSubController()
gyd_base_controller.sub_controller = gyd_sub_controller
gyd_sub_controller.base_controller = gyd_base_controller

gyd_base_controller.connect()

raas_dataset_builder = RaasDatasetBuilder(controller=gyd_sub_controller,
                                          camera_info=VIDEO_INFO)

solar_client = SolarClient(sub_controller=gyd_sub_controller)
solar_thread = threading.Thread(target=lambda:asyncio.run(solar_client.main_loop()),
                                                          daemon=True)
solar_thread.start()

# Keep process alive
while True:
    time.sleep(5)

