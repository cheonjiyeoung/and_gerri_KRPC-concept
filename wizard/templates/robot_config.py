### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": None,
    "model": None,
    "category": None,
    "api_key": None
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    'front_camera':{
        'name':'front_cam',
        'model': 'logitech_c922',
        'source': 0,
        'width': 640,
        'height': 480,
        'fps': 30,
        'pose':{'coordinate':{'x':None,'y':None,'z':None},
                'rpy':{'r':None,'p':None,'y':None}},
        'dFOV':79
    },
    'rear_camera':{
        'name':'rear_cam',
        'model': 'logitech_c922',
        'source': 2,
        'width': 640,
        'height': 480,
        'fps': 30,
        'pose':{'coordinate':{'x':None,'y':None,'z':None},
                'rpy':{'r':None,'p':None,'y':None}},
        'dFOV':79
    }
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}