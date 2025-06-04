import asyncio
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), '../..')))

from _and_.keti_rtc.webrtc_operator_command import WebRtcOperatorCommand
from _and_.keti_rtc.webrtc_operator_video import WebRtcOperatorVideo

SERVER_HOST = '175.126.123.199'
SERVER_PORT = 9980


def create_channels(operator_info, command=None, video_info=None, audio_info=None, password=''):
    operator_id = operator_info['id']
    loop = asyncio.get_event_loop()
    channels = {}

    # 명령 채널
    if command:
        cmds = [command] if isinstance(command, str) else command
        for name in cmds:
            command_channel = WebRtcOperatorCommand(robot_info={'id': operator_id}, operator_info={'id': operator_id, 'password': password})
            command_channel.connect()
            full_id = f'{operator_id}_{name}'
            channels[full_id] = command_channel

    # 비디오 채널
    if video_info:
        for name, cfg in video_info.items():
            video_channel = WebRtcOperatorVideo(robot_info={'id': operator_id}, operator_info={'id': operator_id, 'password': password}, video_info=name)
            video_channel.connect()
            full_id = f'{operator_id}_{name}'
            channels[full_id] = video_channel


    return channels
