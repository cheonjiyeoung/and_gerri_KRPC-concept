import os
import sys

CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from _and_.keti_rtc.webrtc_command_channel import WebRtcCommandChannel
from _and_.keti_rtc.webrtc_video_channel import WebRtcVideoChannel
from _and_.keti_rtc.webrtc_audio_channel import WebRtcAudioChannel
import asyncio
import time

from _and_.keti_rtc.cam_manager import CameraManager
from _and_.keti_rtc.video_streamer import VideoStreamer
from _and_.keti_rtc.stereo_camera_combiner import StereoCameraCombiner
from _and_.keti_rtc.stereo_vr_processor import StereoVROption, StereoVRProcessor

from gerri.robot.gerri_config import *

# from gerri.robot.robot_tools.manipulator_controller import ManipulatorController

SERVER_HOST = "175.126.123.199"
SERVER_PORT = "8180"

# left_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_LEFT, robot_model=ROBOT_MODEL)
# right_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_RIGHT, robot_model=ROBOT_MODEL)
# pan_tilt = PanTiltController(robot_name=PAN_TILT_NAME, use_port=PAN_TILT_USB)

# robot_group = {
#             PUPPET_ARM_NAME_LEFT: left_piper,
#             PUPPET_ARM_NAME_RIGHT: right_piper,
#             # PAN_TILT_NAME: pan_tilt,
# }

robot_group = None

command_channel = WebRtcCommandChannel(robot_id=ROBOT_ID + "_command",
                                   password="",
                                   server_host=SERVER_HOST,
                                   server_port=SERVER_PORT,
                                   # camera=DummyCamera(),
                                   loop=asyncio.get_event_loop()
                                       )


# stereo_camera = StereoCameraCombiner(cam1_index=1, cam2_index=2, width=1920, height=1080)
# stereo_camera_stream = StereoCameraStreamTrack(stereo_camera)

# cam1 = CV2StreamCamera(camera_name='front_cam', camera_index=2, width=1920, height=1080, fps=30)
# cam2 = CameraManager(camera_name='rear_cam', camera_index=0, width=9999, height=9999, fps=30)
# cam2.start()
# cam2_video_streamer = VideoStreamer(cam2)

cam0 = CameraManager(camera_name='vr_cam', camera_index=1, width=9999, height=9999, fps=30)
cam0.start()

# cam1 = CameraManager(camera_name='left_cam', camera_index=2, width=9999, height=9999, fps=30)
# cam1.start()
# cam2 = CameraManager(camera_name='right_cam', camera_index=0, width=9999, height=9999, fps=30)
# cam2.start()

# combined_cam = StereoCameraCombiner(cam1, cam2, width=1920, height=1080, fps=30)
# combined_cam_video_streamer = VideoStreamer(combined_cam)


# 1. 스테레오 카메라 결합 (raw 프레임 제공)
# combined_cam = StereoCameraCombiner(cam1, cam2)

# 2. VR 옵션 구성
vr_option = StereoVROption(
    target_aspect_ratio="2:1",
    # resize_width_px=1280,
    # resize_height_px=720,
    interocular_adjust=-100,      # 좌우 크롭해서 거리 좁힘
    # shift_left=5,                # 추가 미세 조정
    # shift_right=-5,
    zoom_factor=1.0
)

# 3. VR 프로세서 적용
vr_processed_cam = StereoVRProcessor(cam0, option=vr_option)

# 4. 스트리머 생성
combined_cam_video_streamer = VideoStreamer(vr_processed_cam)





# stereo_camera_stream = CV2StreamCamera(camera_name='stereo_camera', camera_index=0, width=2000, height=1000, fps=30)

# video_channel_1_id = ROBOT_ID + "_video_1"
video_channel_2_id = ROBOT_ID + "_video_2"

# video_channel_1 = WebRtcVideoChannel(
#         robot_id=video_channel_1_id,
#         password="",
#         server_host="175.126.123.199",
#         server_port="8180",
#         camera=cam1,
#     )
#
# import cv2
#
# streamer = StereoCameraStreamer(
#     cam1_index=2,
#     cam2_index=3,
#     width=1280,
#     height=720,
#     fps=30,
#     rotate1=cv2.ROTATE_90_COUNTERCLOCKWISE,
#     rotate2=cv2.ROTATE_90_CLOCKWISE
# )


video_channel_2 = WebRtcVideoChannel(
        robot_id=video_channel_2_id,
        password="",
        server_host="175.126.123.199",
        server_port="8180",
        camera=combined_cam_video_streamer,
    )



video_channel_group = {
    # video_channel_1_id: video_channel_1,
    video_channel_2_id: video_channel_2,
}

# video_channel = None

# audio_channel = WebRtcAudioChannel(
#     robot_id=ROBOT_ID + "_audio",
#     password="",
#     server_host="175.126.123.199",
#     server_port="8180"
# )

audio_channel = None

class GerriRobot:
    def __init__(self, robot, command_channel=None, video_channel=None, audio_channel=None):
        self.controller = robot
        self.command_channel = command_channel
        self.video_channel = video_channel
        self.audio_channel = audio_channel


    def connect(self):
        try:
            for controller in self.controller.values():
                controller.connect()
        except Exception as e:
            print(e)

        if self.video_channel is not None:
            try:
                for video_channel in self.video_channel.values():
                    video_channel.connect()
            except Exception as e:
                print(e)
            # self.video_channel.connect()

        if self.audio_channel is not None:
            self.audio_channel.connect()

        if self.command_channel is not None:
            self.command_channel.connect()



if __name__ == "__main__":
    robot = AvatarRobot(robot_group,command_channel,video_channel_group,audio_channel)
    robot.connect()

    while True:
        time.sleep(1)