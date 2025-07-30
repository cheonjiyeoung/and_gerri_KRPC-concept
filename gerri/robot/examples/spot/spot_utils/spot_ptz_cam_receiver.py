# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).
import asyncio
import base64

import requests
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole
from aiortc import RTCConfiguration

import queue
import os
import shutil
import tempfile
from bisect import bisect_left
from bosdyn.client.command_line import Command, Subcommands
import threading
import time
DEFAULT_WEB_REQUEST_TIMEOUT = 10.0
import datetime

class SpotCAMMediaStreamTrack(MediaStreamTrack):
    def __init__(self, track, parent_client):  # ✅ client 객체를 직접 받음
        super().__init__()
        self.track = track
        self.client = parent_client
        self._lock = threading.Lock()

    async def recv(self):
        frame = await self.track.recv()
        with self._lock:
            self.client.last_frame = frame  # ✅ 실제 client 객체에 직접 저장
        return frame



class SpotWebRTCClient:

    def __init__(self, hostname, sdp_port, sdp_filename, cam_ssl_cert, token, rtc_config,
                 media_recorder=None, recorder_type=None, auto_start=True):
        self.pc = RTCPeerConnection(configuration=rtc_config)

        # self.video_frame_queue = asyncio.Queue()
        # self.frame_queue = queue.Queue()
        self.last_frame=None
        
        self.audio_frame_queue = asyncio.Queue()

        self.hostname = hostname
        self.token = token
        self.sdp_port = sdp_port
        self.media_recorder = media_recorder
        self.media_black_hole = None
        self.recorder_type = recorder_type
        self.sdp_filename = sdp_filename
        self.cam_ssl_cert = cam_ssl_cert
        self.sink_task = None
        self.last_frame = None

    def get_bearer_token(self, mock=False):
        if mock:
            return 'token'
        return self.token
    def get_sdp_offer_from_spot_cam(self, token):
        headers = {'Authorization': f'Bearer {token}'}
        server_url = f'https://{self.hostname}:{self.sdp_port}/{self.sdp_filename}'
        
        response = requests.get(server_url, verify=self.cam_ssl_cert, headers=headers,
                                timeout=DEFAULT_WEB_REQUEST_TIMEOUT)
        result = response.json()

        try:
            sdp_offer_raw = base64.b64decode(result['sdp']).decode()
            return result['id'], sdp_offer_raw
        except Exception as e:
            print("Failed to decode SDP:")
            print(e)
            return None, None

    def send_sdp_answer_to_spot_cam(self, token, offer_id, sdp_answer):
        headers = {'Authorization': f'Bearer {token}'}
        server_url = f'https://{self.hostname}:{self.sdp_port}/{self.sdp_filename}'

        payload = {'id': offer_id, 'sdp': base64.b64encode(sdp_answer).decode('utf8')}
        r = requests.post(server_url, verify=self.cam_ssl_cert, json=payload, headers=headers,
                          timeout=DEFAULT_WEB_REQUEST_TIMEOUT)
        if r.status_code != 200:
            raise ValueError(r)

    async def start(self):
        # first get a token
        try:
            token = self.get_bearer_token()
        except:
            token = self.get_bearer_token(mock=True)

        offer_id, sdp_offer = self.get_sdp_offer_from_spot_cam(token)

        @self.pc.on('icegatheringstatechange')
        def _on_ice_gathering_state_change():
            print(f'ICE gathering state changed to {self.pc.iceGatheringState}')

        @self.pc.on('signalingstatechange')
        def _on_signaling_state_change():
            print(f'Signaling state changed to: {self.pc.signalingState}')

        @self.pc.on('icecandidate')
        def _on_ice_candidate(event):
            print(f'Received candidate: {event.candidate}')

        @self.pc.on('iceconnectionstatechange')
        async def _on_ice_connection_state_change():
            print(f'ICE connection state changed to: {self.pc.iceConnectionState}')

            if self.pc.iceConnectionState == 'checking':
                self.send_sdp_answer_to_spot_cam(token, offer_id,
                                                 self.pc.localDescription.sdp.encode())

        @self.pc.on('track')
        def _on_track(track):
            print(f'Received track: {track.kind}')

            if self.media_recorder:
                if track.kind == self.recorder_type:
                    self.media_recorder.addTrack(track)
                else:
                    # We only care about the track we are recording.
                    self.media_black_hole = MediaBlackhole()
                    self.media_black_hole.addTrack(track)
                    loop = asyncio.get_event_loop()
                    self.sink_task = loop.create_task(self.media_black_hole.start())
            else:
                if track.kind == 'video':
                    video_track = SpotCAMMediaStreamTrack(track,self)
                    # video_track = SpotCAMMediaStreamTrack(track,self.frame_queue)
                    # video_track = SpotCAMMediaStreamTrack(track)
                    video_track.kind = 'video'
                    self.pc.addTrack(video_track)

                if track.kind == 'audio':
                    self.media_recorder = MediaBlackhole()
                    self.media_recorder.addTrack(track)
                    loop = asyncio.get_event_loop()
                    self.sink_task = loop.create_task(self.media_recorder.start())

        desc = RTCSessionDescription(sdp_offer, 'offer')
        await self.pc.setRemoteDescription(desc)

        sdp_answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(sdp_answer)

        
    def get_frame(self):
        return self.last_frame
        
