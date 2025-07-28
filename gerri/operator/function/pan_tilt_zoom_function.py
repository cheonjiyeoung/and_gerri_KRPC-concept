from pubsub import pub

import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.function import pan_tilt_zoom_command


class PTZFunction:
    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def pan_tilt(self, pan_tilt_angle, target='all'):
        command = pan_tilt_zoom_command.pan_tilt(pan_tilt_angle=pan_tilt_angle, target=target)
        self.send_message(command)

    def pan_tilt_step(self, pan_tilt_angle_step, target='all'):
        command = pan_tilt_zoom_command.pan_tilt_step(pan_tilt_angle_step=pan_tilt_angle_step, target=target)
        self.send_message(command)

    def zoom(self, zoom_factor, target='all'):
        command = pan_tilt_zoom_command.zoom(zoom_factor=zoom_factor, target=target)
        self.send_message(command)

    def pan_tilt_zoom(self, pan_tilt_zoom_list, target='all'):
        command = pan_tilt_zoom_command.pan_tilt_zoom(pan_tilt_zoom_list=pan_tilt_zoom_list, target=target)
        self.send_message(command)
