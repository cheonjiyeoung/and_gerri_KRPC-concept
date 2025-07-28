ROBOT_INFO = {
    "id": "hello_universe",
    "model": "gerri",
    "category": "sample",
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"index": 0, "width": 1920, "height": 1080},
    # "rear": {"index": 2, "width": 1280, "height": 720},
}
VIDEO_CHANNEL = "front_cam"

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}
AUDIO_CHANNEL = "audio"


### OPERATOR
# Operator account credentials for system login

OPERATOR_INFO = {
    'id': "sunfish",
    'password': 'sunfish',
}

MASTER_ARM_USB_LEFT         = {
    'id': 'puppet_left',
    'port':'/dev/ttyUSB0',
    'baudrate':1000000,
    'joint_limit': [[-30, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}
MASTER_ARM_USB_RIGHT        = {
    'id': 'puppet_right',
    'port':'/dev/ttyUSB1',
    'baudrate':1000000,
    'joint_limit': [[-30, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}
