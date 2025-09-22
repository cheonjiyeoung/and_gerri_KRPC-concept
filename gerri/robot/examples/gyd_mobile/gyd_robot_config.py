### ROBOT
# Defines robot identity, type and classification


ROBOT_INFO = {
    "id": "gyd_mobile_01",
    "model": "gyd_mobile",
    "category": "sample",
    "api_key": "d1ae52308839b4c034f7a5f2c46f477a"
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    'front_camera':{
        'name':'front_camera',
        'model': 'logitech_c922',
        'source': 0,
        'width': 640,
        'height': 480,
        'fps': 30,
        'pose':{'coordinate':{'x':180,'y':0,'z':440},
                'rpy':{'r':0,'p':0,'y':0}},
        'dFOV':79
    },
    'rear_camera':{
        'name':'rear_camera',
        'model': 'logitech_c922',
        'source': 2,
        'width': 640,
        'height': 480,
        'fps': 30,
        'pose':{'coordinate':{'x':180,'y':-540,'z':440},
                'rpy':{'r':0,'p':0,'y':180}},
        'dFOV':79
    }
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}


GYD_INFO = {
    "map_h" : 638,
    "map_w" : 1477,
    "map_resolution" : 0.05,
    "origin" : {"x":-1.81574,
                "y":-13.65},

}

SOLAR_INFO = {
    "ccs_coords":{
        "1F":None,
        "2F":[[-0.63, -3.57],
                [1.56, -0.3],
                [29.52, -3.57]],

        "3F":[[845,587],
                [1274,391],
                [1070,332]]
                },

    "server_host" : {
        "addr" : "125.131.105.165",
        "port" : 10086
    }
}