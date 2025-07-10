# WaveShare Robotaxi - Complete Pi Setup Command Sheet

## Pre-Setup Checklist
- [ ] Raspberry Pi 4B (4GB or 8GB) with Ubuntu 22.04 or Raspberry Pi OS Bookworm
- [ ] MicroSD card (32GB+ recommended)
- [ ] Pi Camera Module or USB camera
- [ ] 4x Ultrasonic sensors (HC-SR04 or similar)
- [ ] 12V battery pack for chassis
- [ ] Wi-Fi connection
- [ ] SSH enabled on Pi

## Complete Setup Commands

| Step | Purpose | Command to Run | Expected Output/Notes |
|------|---------|----------------|----------------------|
| 1 | SSH into Pi | `ssh pi@<your-pi-ip>` | Replace with your Pi's IP address |
| 2 | Update system | `sudo apt update && sudo apt upgrade -y` | May take 5-10 minutes |
| 3 | Install git | `sudo apt install -y git` | If not already installed |
| 4 | Clone repository | `cd ~ && git clone https://github.com/<your-username>/WaveShare.git` | Replace with your actual GitHub username |
| 5 | Navigate to project | `cd ~/WaveShare` | |
| 6 | Make installer executable | `chmod +x setup_pi_robotaxi.sh` | |
| 7 | **RUN MAIN INSTALLER** | `sudo ./setup_pi_robotaxi.sh` | **Takes 15-20 minutes** - Installs ROS2, Nav2, SLAM-Toolbox, YOLO weights, builds workspace |
| 8 | Reboot Pi | `sudo reboot` | Wait 60 seconds before reconnecting |
| 9 | Reconnect SSH | `ssh pi@<your-pi-ip>` | |
| 10 | Check autostart service | `systemctl status robotaxi` | Should show "active (running)" |
| 11 | Source ROS environment | `source /opt/ros/humble/setup.bash && source ~/WaveShare/ros2_ws/install/setup.bash` | |
| 12 | Verify ROS topics | `ros2 topic list` | Should show: /cmd_vel_safe, /odom, /usb_cam/image_raw, /lane_mask, /traffic_light, /sign_detections, /safety_status |

## Testing Commands

| Test | Command | What to Expect |
|------|---------|----------------|
| **Safety System** | `ros2 topic echo /safety_status` | JSON with sensor distances and override state |
| **Camera Feed** | `ros2 run image_view image_view --ros-args -r image:=/usb_cam/image_raw` | Live camera window (needs display) |
| **Traffic Light Detection** | `ros2 topic echo /traffic_light` | "red", "green", "yellow", or "none" |
| **Sign Detection** | `ros2 topic echo /sign_detections` | Detection array with confidence scores |
| **Manual Control** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Use WASD keys to drive (safety override active) |
| **Battery Status** | `ros2 topic echo /battery_status` | Voltage and charge level |

## Navigation Testing (from laptop)

| Step | Command | Notes |
|------|---------|-------|
| 1 | Set ROS domain | `export ROS_DOMAIN_ID=0` | Run on your laptop |
| 2 | Launch RViz | `source /opt/ros/humble/setup.bash && rviz2` | |
| 3 | Add tools | In RViz: Add "Map", "RobotModel", "Nav2 Goal" | |
| 4 | Set frame | Set "Fixed Frame" = `map` | |
| 5 | Send goal | Click "2D Nav Goal" → drag on map | Robot drives to goal |

## Troubleshooting Commands

| Issue | Command | Purpose |
|-------|---------|---------|
| Service not running | `sudo systemctl start robotaxi` | Start the autostart service |
| View service logs | `journalctl -u robotaxi -f` | Real-time service logs |
| Stop autostart | `sudo systemctl stop robotaxi` | Disable automatic startup |
| Manual launch | `source /opt/ros/humble/setup.bash && source ~/WaveShare/ros2_ws/install/setup.bash && ros2 launch robotaxi_launch robotaxi_full_launch.py` | Launch with visible logs |
| Rebuild workspace | `cd ~/WaveShare/ros2_ws && colcon build --symlink-install` | If you modify code |
| Check ROS installation | `ros2 --version` | Verify ROS2 is installed |
| List running nodes | `ros2 node list` | See all active ROS nodes |

## Hardware Verification

| Component | Test Command | Expected Result |
|-----------|--------------|-----------------|
| **Camera** | `ls /dev/video*` | Should show /dev/video0 |
| **Ultrasonic Sensors** | `ros2 topic echo /safety_status` | Distance values > 0 |
| **Motors** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Wheels respond to WASD |
| **Wi-Fi** | `iwconfig` | Shows connected network |
| **Battery** | `ros2 topic echo /battery_status` | Voltage reading |

## What Gets Installed

✅ **ROS 2 Humble** - Core robotics framework  
✅ **Navigation2** - Path planning and obstacle avoidance  
✅ **SLAM-Toolbox** - Mapping and localization  
✅ **Ultralytics YOLO** - Traffic sign and light detection  
✅ **Mediapipe** - Hand/face/pose detection  
✅ **OpenCV** - Computer vision processing  
✅ **Python dependencies** - All required libraries  
✅ **YOLO weights** - Pre-trained models for signs and lights  
✅ **Systemd service** - Automatic startup on boot  

## Post-Setup Features

After running these commands, your robotaxi will:

🚗 **Drive autonomously** - Follow lanes or navigate to goals  
🛑 **Obey traffic rules** - Stop at signs and lights  
🛡️ **Stay safe** - Never hit obstacles closer than 20cm  
📱 **Accept commands** - Via RViz, tele-op, or future taxi API  
🔄 **Auto-restart** - Boots into driving mode automatically  
📊 **Report status** - Battery, safety, detections via ROS topics  

## Next Steps

1. **Test basic functionality** - Use the testing commands above
2. **Calibrate sensors** - Adjust ultrasonic distances if needed
3. **Create a map** - Use SLAM-Toolbox to map your driving area
4. **Add taxi features** - Ride requests, voice, payment (future)

---

**Note**: All commands assume you're working as user `pi`. If using a different username, replace `pi` with your username in SSH commands. 