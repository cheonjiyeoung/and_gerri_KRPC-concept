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

MASTER_ARM_LEFT_INFO = {
    'target': 'puppet_left',
    'master_model': 'piper',
    'port':'/dev/ttyUSB0',
    'joint_limit': [[-90, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}

MASTER_ARM_LEFT_INFO = {
    'target': 'puppet_left',
    'master_model': 'dynamixel',
    'port':'/dev/ttyUSB0',
    'baudrate':1000000,
    'n_dxl': 7,
    'joint_limit': [[-90, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}
MASTER_ARM_RIGHT_INFO = {
    'target': 'puppet_right',
    'master_model': 'dynamixel',
    'port':'/dev/ttyUSB1',
    'baudrate':1000000,
    'n_dxl': 7,
    'joint_limit': [[-90, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}
