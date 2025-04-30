#!/bin/bash

set -e  # 에러나면 종료

echo "📦 [1/5] 시스템 패키지 설치 (Python 3.10 + 빌드 툴)..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-dev python3-pip portaudio19-dev build-essential

echo "🐍 [2/5] Python 가상환경 생성..."
python3.10 -m venv venv

echo "🔁 [3/5] 가상환경 활성화 및 pip 업그레이드..."
source venv/bin/activate
pip install --upgrade pip

echo "📜 [4/5] Python requirements 설치..."
pip install -r requirements.txt

echo ""
echo "✅ 완료: 환경 준비가 끝났습니다!"
echo "👉 다음 명령어로 가상환경 활성화:"
echo "   source venv/bin/activate"
