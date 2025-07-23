### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": "construction_doosan",
    "model": "doosanM1509",
    "category": "sample",
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "realsense_D435if": {"index": 0, "width": 480, "height": 1280},
    # "rear": {"index": 2, "width": 1280, "height": 720},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

### OPERATOR
# Operator account credentials for system login

OPERATOR_INFO = {
    'id': "salmon",
    'password': 'keti1234',
}