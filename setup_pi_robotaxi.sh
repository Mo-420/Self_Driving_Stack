#!/bin/bash
# setup_pi_robotaxi.sh ----------------------------------------------------
# One-time installer that turns a fresh Raspberry Pi (Ubuntu 22.04) into a
# ready-to-run WaveShare Robotaxi with ROS 2, Navigation 2, SLAM-Toolbox and
# perception models.
#
# Usage:  ssh pi@<pi-ip> "bash -s" < setup_pi_robotaxi.sh
# -------------------------------------------------------------------------
set -e

# 1. System update ---------------------------------------------------------
sudo apt update && sudo apt upgrade -y

# 2. ROS 2 Humble ----------------------------------------------------------
if ! grep -q "ros2.list" /etc/apt/sources.list.d/; then
  sudo apt install -y curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-rosbridge-suite \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-vision-msgs ros-humble-cv-bridge

# 2b. Additional ROS camera driver
sudo apt install -y ros-humble-v4l2-camera

# 3. Python deps -----------------------------------------------------------
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics==8.1.0 mediapipe==0.10.9 \
  numpy opencv-python

# 3b. Extra Python libs
python3 -m pip install tf-transformations --break-system-packages || true

# 4. Clone your repo (assume /home/$USER/WaveShare already copied)
# cd ~/WaveShare     # uncomment if necessary

# 5. Download YOLO weights -------------------------------------------------
mkdir -p ~/WaveShare/models
if [ ! -f ~/WaveShare/models/yolov8n-signs.pt ]; then
  wget -O ~/WaveShare/models/yolov8n-signs.pt \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
fi

# 5b. Download traffic-light YOLO weights
if [ ! -f ~/WaveShare/models/yolov8n-traffic.pt ]; then
  wget -O ~/WaveShare/models/yolov8n-traffic.pt \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
fi

# 6. Build ROS 2 workspace -------------------------------------------------
cd ~/WaveShare/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 7. Add workspace to bashrc ----------------------------------------------
if ! grep -q "source ~/WaveShare/ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/WaveShare/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# 7b. Add project root to PYTHONPATH for every shell
if ! grep -q "PYTHONPATH.*WaveShare" ~/.bashrc; then
  echo "export PYTHONPATH=\$PYTHONPATH:~/WaveShare" >> ~/.bashrc
fi

echo "\n✅ Robotaxi software installed.  Reboot then run:"
echo "   ros2 launch robotaxi_launch robotaxi_launch.py" 