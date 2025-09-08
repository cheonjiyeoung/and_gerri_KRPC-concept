#!/bin/bash

PROJECT_DIR="~/dev/and_gerri/"

bash ~/dev/and_gerri/gerri/robot/examples/piper/can_activate.sh puppet_left 1000000 "3-3.1:1.0"
bash ~/dev/and_gerri/gerri/robot/examples/piper/can_activate.sh puppet_right 1000000 "3-3.2:1.0"


source ~/dev/and_gerri/venv/bin/activate

sleep 5


python ~/dev/and_gerri/gerri/robot/examples/piper/piper_dual_robot.py
