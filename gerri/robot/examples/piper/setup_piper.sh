#!/bin/bash

PROJECT_DIR="~/dev/avatar"

sudo apt update && sudo apt install -y can-utils ethtool

python3 -m venv "$PROJECT_DIR/venv"
source "$PROJECT_DIR/venv/bin/activate"

# 의존성 설치
pip install -r "$PROJECT_DIR/avatar_piper/robot/requirement.txt"