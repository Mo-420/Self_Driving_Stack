# Quick Start Guide - WaveShare Robotaxi

## 🚀 Get Running in 5 Minutes

### Prerequisites
- Raspberry Pi 4B with Raspberry Pi OS Lite (64-bit)
- Waveshare Robot-Chassis MP
- Wi-Fi connection
- SSH access to Pi

### Step 1: Flash & Boot
```bash
# Use Raspberry Pi Imager to flash OS Lite (64-bit)
# Enable SSH and configure Wi-Fi during flashing
# Boot Pi and note the IP address
```

### Step 2: Connect & Update
```bash
# SSH into your Pi
ssh pi@<your_pi_ip>
# Default password: raspberry

# Update system
sudo apt update && sudo apt full-upgrade -y
```

### Step 3: One-Command Setup
```bash
# Download and run the automated setup
curl -O https://raw.githubusercontent.com/your-repo/setup_robotaxi.sh
chmod +x setup_robotaxi.sh
sudo ./setup_robotaxi.sh
```

### Step 4: Reboot & Access
```bash
# Reboot the Pi
sudo reboot

# Wait 30 seconds, then access:
# Control Dashboard: http://<your_pi_ip>:5000
# JupyterLab: http://<your_pi_ip>:8888
```

## 🔧 Manual Setup (Alternative)

If the automated script doesn't work:

```bash
# Install dependencies
sudo apt install -y git python3-venv python3-pip

# Clone Waveshare repo
git clone https://github.com/waveshareteam/ugv_rpi.git
cd ugv_rpi

# Make scripts executable
sudo chmod +x setup.sh autorun.sh

# Run setup
sudo ./setup.sh
./autorun.sh

# Enable interfaces
sudo raspi-config nonint do_camera 0
sudo raspi-config nonint do_serial 0
sudo raspi-config nonint do_serial_hw 1

# Reboot
sudo reboot
```

## 🎮 Using the Control Dashboard

### Basic Controls
- **Forward/Backward**: Use arrow keys or on-screen buttons
- **Left/Right**: Turn the vehicle
- **Stop**: Emergency stop button
- **Speed**: Adjustable speed slider

### Video Streaming
- Live camera feed appears automatically
- Click video area to toggle fullscreen
- Video quality can be adjusted in settings

### Advanced Features
- **Manual Mode**: Direct control via web interface
- **Auto Mode**: Basic autonomous features (Phase 2)
- **Settings**: Configure motor parameters, camera settings

## 🔍 Troubleshooting Quick Fixes

### Motors Not Moving
```bash
# Check UART connection
ls /dev/ttyAMA0

# Test serial communication
echo "test" | sudo tee /dev/ttyAMA0
```

### Camera Not Working
```bash
# Enable camera
sudo raspi-config nonint do_camera 0

# Test camera
raspistill -o test.jpg
```

### Web Dashboard Not Loading
```bash
# Check if service is running
sudo systemctl status ugv_control

# Restart service
sudo systemctl restart ugv_control
```

### Can't Connect to Pi
```bash
# Find Pi IP address
hostname -I

# Check if SSH is enabled
sudo systemctl status ssh
```

## 📱 Mobile Access

The web dashboard is mobile-responsive:
- Open browser on phone/tablet
- Navigate to `http://<pi_ip>:5000`
- Use touch controls for driving

## 🎯 Next Steps

After successful setup:
1. **Test basic movement** - Drive forward/backward
2. **Test camera** - Verify video streaming
3. **Test auto-start** - Reboot and verify dashboard loads
4. **Begin Phase 2** - Add line following, obstacle avoidance

## 📞 Getting Help

- Check the full [README.md](README.md) for detailed documentation
- Review [WIRING_GUIDE.md](WIRING_GUIDE.md) for connection issues
- Visit [Waveshare UGV Documentation](https://www.waveshare.com/wiki/UGV_RPi)

---

**Happy Robotaxi Development! 🚗✨** 