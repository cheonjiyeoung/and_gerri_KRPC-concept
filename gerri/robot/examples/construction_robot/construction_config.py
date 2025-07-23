### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": "construction_doosan",
    "model": "doosanM1509",
    "category": "sample",
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "realsense_D435if_rgb": {"source": None, "width": 480, "height": 1280},
    "realsense_D435if_depth": {"source": None, "width": 480, "height": 1280}
    "3D_cam": {"source": 0, "width": 9999, "height": 9999},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    #"audio": {"input": "default", "output": "default"},
}

### OPERATOR
# Operator account credentials for system login

OPERATOR_INFO = {
    'id': "salmon",
    'password': 'keti1234',
}
