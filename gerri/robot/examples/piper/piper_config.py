ROBOT_INFO = {
    "id": "gerri_piper_01",
    "model": "piper",
    "category": "manipulator",
}

PUPPET_ARM_LEFT = {
    "id": "puppet_left",
    "model": "piper",
    "category": "manipulator",
}

PUPPET_ARM_RIGHT = {
    "id": "puppet_right",
    "model": "piper",
    "category": "manipulator",
}

PAN_TILT = {
    "id": "pan_tilt",
    "model": "pan_tilt",
    "category": "",
    "usb": "/dev/ttyUSB0"
}

VIDEO_INFO = {
    "front_cam": {"index": 0, "width": 1920, "height": 1080},
    # "rear": {"index": 2, "width": 1280, "height": 720},
}

AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}
