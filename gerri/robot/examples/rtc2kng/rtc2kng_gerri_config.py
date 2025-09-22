ROBOT_INFO = {
    "id": "gc_dual",
    "model": "doosan_m1509_dual",
    "category": "manipulator",
    "ip": "192.168.10.119",
    "port": 502,
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 2560, "height": 720},
    # "rear": {"source": 2, "width": 1280, "height": 720},
}

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}

SERVER_INFO = {
    # "server_ip": '125.131.105.165',
    "server_ip": '172.20.1.250',
    "server_port": 25000,
    "room_id": 'test_room',
}