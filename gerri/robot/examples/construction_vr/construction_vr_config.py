ROBOT_INFO = {
    "id": "gc_dual",
    "model": "gerri",
    # "model": "doosan_m1509_dual",
    "category": "manipulator",
    "ip": "192.168.10.119",
    "port": 502,
}

from _and_.rtc2kng.ldz_camera_manager import LDZCameraManager
# Camera settings: device index and resolution
VIDEO_INFO = {
    'zed_vr_cam': {'manager': LDZCameraManager, 'source': 0,
                   'input_width': 3840, 'input_height': 1080,
                   'output_width': 1280, 'output_height': 360,
                   'fps': 30
                   },
    # "left": {"source": 2, "width": 640, "height": 480},
    # "right": {"source": 4, "width": 640, "height": 480},
    # "rear": {"source": 2, "width": 1280, "height": 720},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

SERVER_INFO = {
    "server_ip": '125.131.105.165',
    # "server_ip": '172.20.1.250',
    "server_port": 25000,
    "room_id": 'test_room',
}