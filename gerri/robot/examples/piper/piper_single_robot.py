import time
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.piper.piper_single_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

### Network system setting ###

# 🚀 데몬 실행
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO
)

daemon.connect()
# daemon.run_forever()

### Robot system setting

from gerri.robot.examples.piper.piper_base_controller import PiperBaseController

piper = PiperBaseController(ROBOT_INFO)
piper.connect()


while True:
    time.sleep(1)