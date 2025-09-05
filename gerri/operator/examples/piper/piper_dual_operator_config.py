ROBOT_INFO = {
    "id": "gerri_piper_01",
    "model": "piper",
    "category": "manipulator",
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 1920, "height": 1080},
    # "rear": {"source": 2, "width": 1280, "height": 720},
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
    'id': 'master_left',
    'target': 'puppet_left',
    'master_model': 'piper',
    'joint_limit': [[-105, 105], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}


MASTER_ARM_RIGHT_INFO = {
    'id': 'master_right',
    'target': 'puppet_right',
    'master_model': 'piper',
    'joint_limit': [[-105, 105], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
}

# MASTER_ARM_LEFT_INFO = {
#     'target': 'puppet_left',
#     'master_model': 'dynamixel',
#     'port':'/dev/ttyUSB0',
#     'baudrate':1000000,
#     'n_dxl': 7,
#     'joint_limit': [[-90, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
# }
# MASTER_ARM_RIGHT_INFO = {
#     'target': 'puppet_right',
#     'master_model': 'dynamixel',
#     'port':'/dev/ttyUSB1',
#     'baudrate':1000000,
#     'n_dxl': 7,
#     'joint_limit': [[-90, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]],
# }
