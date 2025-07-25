### ROBOT
# Defines robot identity, type and classification

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
    'id': "salmon",
    'password': 'keti1234',
}

