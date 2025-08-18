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


from bosdyn.api.spot_cam import ptz_pb2
from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.compositor import CompositorClient
from gerri.robot.examples.spot.spot_controller import SpotController
from threading import Thread

class SpotPTZ(SpotController):
    def __init__(self,spot_info):
        super().__init__(spot_info)
        # init pan = 145
        # init tilt = 0
        # ptz name = mech
        self.ptz = self.robot.ensure_client(PtzClient.default_service_name)
        self.compositor_client = self.robot.ensure_client(CompositorClient.default_service_name)
        self.lightning_client = self.robot.ensure_client(LightingClient.default_service_name)
        result = self.compositor_client.set_screen("mech_full")
        # self.spot_ptz_absolute(ptz={"pan":145,"tilt":0,"zoom":1})
        self.ptz_focus_dist = 0.0

    def spot_set_compositor(self,name):
        print(222)
        result = self.compositor_client.set_screen(name)
        print(f"\n\n{result}\n\n")
        return result

    def spot_ptz_get_position(self):
        ptz_desc = ptz_pb2.PtzDescription(name="mech")
        ptz_position = self.ptz.get_ptz_position(ptz_desc)
        return ptz_position

    def spot_ptz_relative(self,pan_step,tilt_step,zoom_step):
        ptz_desc = ptz_pb2.PtzDescription(name="mech")
        current_position = self.spot_ptz_get_position()
        pan = current_position.pan.value
        tilt = current_position.tilt.value
        zoom = current_position.zoom.value
        step_p = pan_step
        step_t = tilt_step
        step_z = zoom_step
        ptz_position = self.ptz.set_ptz_position(ptz_desc,
                                                 pan+step_p,
                                                 tilt+step_t,
                                                 zoom+step_z)
        return ptz_position

    def spot_ptz_absolute(self,pan,tilt,zoom):
        ptz_desc = ptz_pb2.PtzDescription(name="mech")
        ptz_position = self.ptz.set_ptz_position(ptz_desc,
                                                 pan,
                                                 tilt,
                                                 zoom)
        return ptz_position
    
    def spot_ptz_initialize(self):
        resp = self.ptz.initialize_lens()
        return resp

    def spot_ptz_get_focus(self):
        focus_info = self.ptz.get_ptz_focus_state()
        return focus_info
    
    def spot_ptz_set_focus(self,mode,dist=None):
        if mode == "manual":
            self.ptz_focus_mode = "manual"
            ptz_focus = self.ptz.set_ptz_focus_state(
                        ptz_pb2.PtzFocusState.PTZ_FOCUS_MANUAL, dist, None)
        elif mode == "auto":
            self.ptz_focus_mode = "auto"
            ptz_focus = self.ptz.set_ptz_focus_state(
                        ptz_pb2.PtzFocusState.PTZ_FOCUS_AUTO, None, None)  
            
        return ptz_focus
            
    def spot_ptz_set_focus_relative(self,step):
        ptz_focus = self.ptz.set_ptz_focus_state(
                    ptz_pb2.PtzFocusState.PTZ_FOCUS_MANUAL, self.ptz_focus_dist+step, None)
        return ptz_focus
    
    def spot_light(self,level:list):
        result = self.lightning_client.set_led_brightness_async(level)
        return result
    
    def get_spot_light_level(self):
        result = self.lightning_client.get_led_brightness()
        print(f"\n\n current robot_light_level. level : {result}")
        return result