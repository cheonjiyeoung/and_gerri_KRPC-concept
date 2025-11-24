### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": 'gyd_01',
    "model": 'gyd',
    "category": 'Mobile',
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": None, "width": 640, "height": 480},
    "rest_cam": {"source": None, "width": 640, "height": 480},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

### OPERATOR
# Operator account credentials for system login

OPERATOR_INFO = {
    'id': 'asd',
    'password': 'asd',
}

# You can add any constant for your robot