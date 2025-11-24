### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": '__ROBOT_ID__',
    "model": '__ROBOT_MODEL__',
    "category": '__ROBOT_CATEGORY__',
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
    'id': '__RUBBERNECK_ID__',
    'password': '__RUBBERNECK_PASSWARD__',
}

# You can add any constant for your robot