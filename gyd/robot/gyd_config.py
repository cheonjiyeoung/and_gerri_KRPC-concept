### ROBOT
# Defines robot identity, type and classification
ROBOT_INFO = {
    "id": "gyd_01",
    "model": "gyd",
    "category": "Mobile",
    "api_key": "asd"
}

### VIDEO
# Camera settings: device index and resolution
# Top-level key in VIDEO_INFO is the channel name.
# If top-level key = 'front_camera', it appears as <robot_id>_front_cam in RubberNeck.
# If you want to use a custom video source, set your own class at ['source'].
# If you want to use our data-collection service, add more camera data (pose, dFOV).
VIDEO_INFO = {
    "front_camera": {
        "name": "front_cam",
        "model": "logitech_c922",
        "source": 0,
        "width": 640,
        "height": 480,
        "fps": 30,
        "pose": {
            "coordinate": {
                "x": None,
                "y": None,
                "z": None
            },
            "rpy": {
                "r": None,
                "p": None,
                "y": None
            }
        },
        "dFOV": 79
    },
    "rear_camera": {
        "name": "rear_cam",
        "model": "logitech_c922",
        "source": 2,
        "width": 640,
        "height": 480,
        "fps": 30,
        "pose": {
            "coordinate": {
                "x": None,
                "y": None,
                "z": None
            },
            "rpy": {
                "r": None,
                "p": None,
                "y": None
            }
        },
        "dFOV": 79
    }
}

### AUDIO
# Audio I/O devices
AUDIO_INFO = {
    "audio": {
        "input": "default",
        "output": "default"
    }
}

# You can add any constant for your robot
