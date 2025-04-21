import asyncio


import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.webrtc_command_channel import WebRtcCommandChannel
from _and_.keti_rtc.webrtc_video_channel import WebRtcVideoChannel
from _and_.keti_rtc.webrtc_audio_channel import WebRtcAudioChannel

SERVER_HOST = "175.126.123.199"
SERVER_PORT = 8180

def create_command_channels(robot_id, commands, password=""):
    if not commands:
        return {}
    loop = asyncio.get_event_loop()
    commands = [commands] if isinstance(commands, str) else commands
    return {
        f"{robot_id}_{name}": WebRtcCommandChannel(
            robot_id=f"{robot_id}_{name}",
            password=password,
            server_host=SERVER_HOST,
            server_port=SERVER_PORT,
            loop=loop
        ) for name in commands
    }

def create_video_channels(robot_id, video_streamers, password=""):
    if not video_streamers:
        return {}
    loop = asyncio.get_event_loop()
    video_streamers = [video_streamers] if not isinstance(video_streamers, list) else video_streamers
    return {
        f"{robot_id}_video_{getattr(v.source, 'camera_name', f'video_{i}')}":
            WebRtcVideoChannel(
                robot_id=f"{robot_id}_{getattr(v.source, 'camera_name', f'video_{i}')}",
                password=password,
                server_host=SERVER_HOST,
                server_port=SERVER_PORT,
                camera=v,
                loop=loop
            )
        for i, v in enumerate(video_streamers)
    }

def create_audio_channels(robot_id, audios, password=""):
    if not audios:
        return {}
    audios = [audios] if isinstance(audios, str) else audios
    return {
        f"{robot_id}_{name}": WebRtcAudioChannel(
            robot_id=f"{robot_id}_{name}",
            password=password,
            server_host=SERVER_HOST,
            server_port=SERVER_PORT
        ) for name in audios
    }