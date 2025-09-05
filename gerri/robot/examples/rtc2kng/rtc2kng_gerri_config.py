ROBOT_INFO = {
    "id": "gc_dual",
    "model": "doosan_m1509_dual",
    "category": "manipulator",
    "ip": "172.20.1.247",
    "port": 502,
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 1920, "height": 1080},
    # "rear": {"source": 2, "width": 1280, "height": 720},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

SERVER_INFO = {
    # "server_ip": '125.131.105.165',
    "server_ip": '172.20.1.251',
    "server_port": 25100,
    "room_id": 'test_room',
}