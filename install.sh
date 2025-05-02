#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

echo "📦 [1/5] Installing system packages (Python 3.10 + build tools)..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-dev python3-pip portaudio19-dev build-essential

echo "🐍 [2/5] Creating Python virtual environment..."
python3.10 -m venv venv

echo "🔁 [3/5] Activating virtual environment and upgrading pip..."
source venv/bin/activate
pip install --upgrade pip

echo "📜 [4/5] Installing Python requirements..."
pip install -r requirements.txt

echo ""
echo "✅ Done: Environment setup is complete!"
echo "👉 To activate the virtual environment, run:"
echo "   source venv/bin/activate"
