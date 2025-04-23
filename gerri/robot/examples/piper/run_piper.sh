#!/bin/bash


bash ~/dev/avatar/avatar_darm/robot/robot_tools/piper/can_activate.sh puppet_left 1000000 "3-1:1.0"
bash ~/dev/avatar/avatar_darm/robot/robot_tools/piper/can_activate.sh puppet_right 1000000 "1-1:1.0"


source ~/dev/avatar/venv/bin/activate

sleep 5


#python ~/dev/avatar/avatar_darm/robot/webrtc/webrtc_video_channel.py &
#python ~/dev/avatar/avatar_darm/robot/webrtc/webrtc_audio_channel.py &
# sudo chmod 777 /dev/ttyUSB0
python ~/dev/avatar/avatar_darm/robot/avatar_robot.py
# python /home/rb5keti/dev/avatar/avatar_darm/robot/robot_tools/piper/piper_robot.py
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

