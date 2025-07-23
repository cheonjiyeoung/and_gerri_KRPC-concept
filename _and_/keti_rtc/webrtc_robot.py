import asyncio
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.webrtc_command_channel import WebRtcCommandChannel
from _and_.keti_rtc.webrtc_video_channel import WebRtcVideoChannel
from _and_.keti_rtc.webrtc_audio_channel import WebRtcAudioChannel

SERVER_HOST = "175.126.123.199"
SERVER_PORT = 9980

def create_channels(robot_info, command=None, video_info=None, audio_info=None, password=""):
    robot_id = robot_info["id"]
    loop = asyncio.get_event_loop()
    channels = {}

    # Command
    if command:
        cmds = [command] if isinstance(command, str) else command
        for name in cmds:
            full_id = f"{robot_id}_{name}"
            channels[full_id] = WebRtcCommandChannel(
                robot_id=full_id,
                password=full_id,
                server_host=SERVER_HOST,
                server_port=SERVER_PORT,
                loop=loop,
                robot_group_id=robot_id,
                api_key=api_key
            )

    # Video
    if video_info:
        from _and_.keti_rtc.cam_manager import CameraManager
        from _and_.keti_rtc.video_streamer import VideoStreamer
        for name, cfg in video_info.items():
            if type(cfg["source"]) != int:
                cam = cfg["source"]
            else:
                cam = CameraManager(camera_name=name, camera_index=cfg["source"],
                            width=cfg["width"], height=cfg["height"], fps=cfg.get("fps", 30))

            streamer = VideoStreamer(cam)
            full_id = f"{robot_id}_{name}"
            channels[full_id] = WebRtcVideoChannel(
                robot_id=full_id,
                password=full_id,
                server_host=SERVER_HOST,
                server_port=SERVER_PORT,
                camera=streamer,
                loop=loop,
                robot_group_id=robot_id,
                api_key=api_key
            )

    # Audio
    if audio_info:
        for name, cfg in audio_info.items():
            full_id = f"{robot_id}_{name}"
            channels[full_id] = WebRtcAudioChannel(
                robot_id=full_id,
                password=full_id,
                server_host=SERVER_HOST,
                server_port=SERVER_PORT,
                robot_group_id=robot_id,
                api_key=api_key,
                audio_input=cfg.get("input"),
                audio_output=cfg.get("output")
            )

    return channels
