### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": "spot_01",
    "model": "spot",
    "category": "spot",
    "api_key": "cdbbe30562edc879dff2f036a9f54d20",
}

SPOT_INFO = {
    "using_ptz":True,
    "using_arm":False,
    "spot_ip":"192.168.50.3",
    "spot_id":"admin",
    "spot_pwd":"at54voe0pz48"
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 640, "height": 480},
    "rear_cam": {"source": 2, "width": 640, "height": 480},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

GERRI_ROBOT_INFO = {
    "id": "gerri_spot_01",
    "model": "spot",
    "category": "spot",
    "api_key": "48ddac9b6e0616849a844a257ca7468c"
}

GERRI_VIDEO_INFO = {
    "ptz_cam": {"source": None, "width": 1920, "height": 1080},
    "rest_cam": {"source": None, "width": 1280, "height": 720},
}