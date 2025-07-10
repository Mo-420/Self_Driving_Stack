# WaveShare Robotaxi - Miniature Autonomous Vehicle

A Raspberry Pi 4B-powered miniature robotaxi built on the Waveshare Robot-Chassis MP platform.

## Project Overview

This project transforms the Waveshare Robot-Chassis MP into a miniature robotaxi with three development phases:

### Phase 1: Basic Control & Video Streaming ✅
- [x] Motor control from Raspberry Pi
- [x] Live camera feed streaming
- [x] Web dashboard with auto-start
- [x] Headless operation ready

### Phase 2: Basic Autonomy (Coming Soon)
- [ ] Line following capabilities
- [ ] Obstacle avoidance
- [ ] Basic navigation

### Phase 3: Robotaxi UX (Future)
- [ ] Ride request system
- [ ] Voice/QR code check-in
- [ ] ROS-based SLAM navigation
- [ ] Advanced autonomy features

## Hardware Requirements

- **Chassis**: Waveshare Robot-Chassis MP
- **Computer**: Raspberry Pi 4B (4GB or 8GB recommended)
- **Camera**: Compatible with Pi Camera Module
- **Power**: 12V battery pack
- **Connectivity**: Wi-Fi for remote access

## Quick Setup Guide

### Step 0: Flash Raspberry Pi OS
```bash
# Use Raspberry Pi Imager to flash Raspberry Pi OS Lite (64-bit)
# Enable SSH and configure Wi-Fi during flashing
```

### Step 1: Initial Setup
```bash
# SSH into your Pi
ssh pi@<your_pi_ip>
# Default password: raspberry

# Update system
sudo apt update && sudo apt full-upgrade -y
```

### Step 2: Install Dependencies
```bash
# Install development tools
sudo apt install -y git python3-venv python3-pip

# Clone the Waveshare repository
git clone https://github.com/waveshareteam/ugv_rpi.git
cd ugv_rpi

# Make scripts executable
sudo chmod +x setup.sh autorun.sh
```

### Step 3: Run Installation
```bash
# Install all required packages (Flask, OpenCV, GPIO, Jupyter)
sudo ./setup.sh

# Enable auto-start on boot
./autorun.sh
```

### Step 4: Test & Access
```bash
# Reboot the Pi
sudo reboot

# Access the control dashboard
# Open browser: http://<your_pi_ip>:5000

# Or access JupyterLab for development
# Open browser: http://<your_pi_ip>:8888
```

## Wiring Diagram

### Pi ↔ Chassis Connections
- **Pi UART TX** → **Chassis ESP32 RX**
- **Pi UART RX** → **Chassis ESP32 TX**  
- **Pi GND** → **Chassis GND**
- **Pi 5V** → **Chassis 5V** (if needed)

### Camera Connection
- Connect Pi Camera Module to the CSI port on the Pi

## Troubleshooting

### Motors Not Responding
1. Check UART connections (TX→RX, RX→TX)
2. Verify common ground connection
3. Check power supply to chassis
4. Ensure proper serial communication setup

### Camera Not Working
1. Verify camera module connection
2. Enable camera in `raspi-config`
3. Check camera permissions

### Web Dashboard Not Accessible
1. Verify Pi is connected to network
2. Check if Flask service is running: `sudo systemctl status ugv_control`
3. Verify firewall settings

## Development

### File Structure
```
ugv_rpi/
├── app.py              # Main Flask application
├── camera.py           # Camera streaming module
├── motor_control.py    # Motor control interface
├── setup.sh           # Installation script
├── autorun.sh         # Auto-start configuration
└── templates/         # Web UI templates
```

### Customization
- Modify `app.py` for custom control logic
- Update `templates/` for UI changes
- Extend `motor_control.py` for advanced motor features

## Safety Notes

⚠️ **Important Safety Considerations:**
- Always test in a safe, open area
- Keep hands clear of moving parts during testing
- Monitor battery levels to prevent sudden shutdowns
- Ensure emergency stop capability is available

## License

This project is based on Waveshare's UGV RPi repository and follows their licensing terms.

## Support

- [Waveshare UGV Documentation](https://www.waveshare.com/wiki/UGV_RPi)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [Project Issues](https://github.com/your-repo/issues) 