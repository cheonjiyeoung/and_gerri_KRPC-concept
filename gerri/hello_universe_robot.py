import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon

from _and_.keti_rtc.cam_manager import CameraManager
from _and_.keti_rtc.video_streamer import VideoStreamer

from gerri.robot.gerri_config import ROBOT_ID

# 🎥 카메라 구성
cam = CameraManager(camera_name="front_cam", camera_index=0, width=1920, height=1080, fps=30)

streamer = VideoStreamer(cam)

# 🚀 데몬 실행
daemon = AdaptiveNetworkDaemon(
    robot_id=ROBOT_ID,
    backend="ketirtc",
    command="command",
    video=streamer,
    audio="audio"
)

daemon.connect()

