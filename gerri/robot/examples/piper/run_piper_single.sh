#!/bin/bash

bash ~/dev/and_gerri/gerri/robot/examples/piper/can_activate.sh piper_gyd 1000000 "1-2:1.0"

source ~/dev/and_gerri/venv/bin/activate

sleep 5

python ~/dev/and_gerri/gerri/robot/examples/piper/piper_single_robot.py

