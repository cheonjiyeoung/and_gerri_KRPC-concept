#!/bin/bash

PROJECT_DIR="~/dev/and_gerri/"
#TOOL_DIR="$PROJECT_DIR/avatar_darm/robot/webrtc"
#source "$PROJECT_DIR/venv/bin/activate"

#bash ~/dev/avatar/avatar_darm/robot/robot_tools/piper/can_activate.sh puppet_left 1000000 "3-1:1.0"
bash ~/dev/and_gerri/gerri/robot/examples/piper/can_activate.sh right_piper 1000000 "1-3.3:1.0"


source ~/dev/and_gerri/venv/bin/activate

sleep 5


python ~/dev/and_gerri/gerri/robot/examples/piper/piper_robot_mono.py

#PROJECT_DIR="~/dev/avatar"
#TOOL_DIR="$PROJECT_DIR/avatar_darm/robot/webrtc"
#source "$PROJECT_DIR/venv/bin/activate"
#
#./can_activate.sh can0 1000000 &
#
##sleep 10
#cd $TOOL_DIR
#
#
#python "webrtc_command_channel.py"
#
#it does not work tq

