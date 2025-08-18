### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": "spot_01",
    "model": "spot",
    "category": "sample",
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": None, "width": 640, "height": 1080},
    "rest_cam": {"source": None, "width": 1280, "height": 720},
}

GERRI_ROBOT_INFO = {
    "id": "gerri_spot_01",
    "model": "spot",
    "category": "sample",
}
GERRI_VIDEO_INFO = {
    "ptz_cam": {"source": None, "width": 640, "height": 1080},
    "rest_cam": {"source": None, "width": 1280, "height": 720},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

### OPERATOR
# Operator account credentials for system login

OPERATOR_INFO = {
    'id': "manta",
    'password': 'cjy2709@',
}

SPOT_INFO = {"using_arm":False,
             "using_ptz":True,
             "default_pt_relative_step":1.0,
             "default_zoom_relative_step":1.0,
             "default_linear_speed":0.7,
             "default_linear_low_speed":0.3,
             "default_angular_speed":0.7,
             "default_angular_low_speed":0.3,
             "missions": ["KOSPO_TestRoute3","keti_2f-3f_round","keti_3f-2f_round","callbacktest2","callbacktest","iii","test0813-1"]
             }
