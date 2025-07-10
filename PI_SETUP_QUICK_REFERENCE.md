# 🚗 WaveShare Robotaxi - Quick Setup Reference

## ⚡ Essential Commands (Copy-Paste)

```bash
# 1. SSH into Pi
ssh pi@<your-pi-ip>

# 2. Update & install
sudo apt update && sudo apt upgrade -y
sudo apt install -y git

# 3. Get code
cd ~ && git clone https://github.com/<your-username>/WaveShare.git
cd ~/WaveShare

# 4. Run installer (15-20 min)
chmod +x setup_pi_robotaxi.sh
sudo ./setup_pi_robotaxi.sh

# 5. Reboot & verify
sudo reboot
# Wait 60s, then reconnect:
ssh pi@<your-pi-ip>
systemctl status robotaxi
```

## 🔧 Quick Tests

```bash
# Safety system
ros2 topic echo /safety_status

# Manual drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Camera feed
ros2 run image_view image_view --ros-args -r image:=/usb_cam/image_raw

# Traffic lights
ros2 topic echo /traffic_light
```

## 🚨 Troubleshooting

```bash
# Service issues
sudo systemctl start robotaxi
journalctl -u robotaxi -f

# Manual launch
source /opt/ros/humble/setup.bash
source ~/WaveShare/ros2_ws/install/setup.bash
ros2 launch robotaxi_launch robotaxi_full_launch.py
```

## 📱 Navigation (from laptop)

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
rviz2
# Add: Map, RobotModel, Nav2 Goal
# Set Fixed Frame = map
# Click "2D Nav Goal" to send destination
```

---

**✅ After setup: Robot drives autonomously, obeys traffic rules, avoids obstacles, auto-starts on boot** 