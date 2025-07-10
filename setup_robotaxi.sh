#!/bin/bash

# WaveShare Robotaxi Setup Script
# This script automates the setup of the Waveshare UGV RPi system for robotaxi development

set -e  # Exit on any error

echo "🚗 WaveShare Robotaxi Setup Script"
echo "=================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script must be run as root (use sudo)"
    exit 1
fi

# Check if we're on a Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "⚠️  Warning: This script is designed for Raspberry Pi"
    echo "   Continue anyway? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "📦 Step 1: Updating system packages..."
apt update && apt full-upgrade -y

echo "🔧 Step 2: Installing development tools..."
apt install -y git python3-venv python3-pip python3-opencv python3-flask python3-serial

echo "📁 Step 3: Creating project directory..."
cd /home/pi
if [ -d "ugv_rpi" ]; then
    echo "   Repository already exists, updating..."
    cd ugv_rpi
    git pull
else
    echo "   Cloning Waveshare UGV repository..."
    git clone https://github.com/waveshareteam/ugv_rpi.git
    cd ugv_rpi
fi

echo "🔐 Step 4: Making scripts executable..."
chmod +x setup.sh autorun.sh

echo "⚙️  Step 5: Running Waveshare setup..."
./setup.sh

echo "🚀 Step 6: Enabling auto-start..."
./autorun.sh

echo "📷 Step 7: Enabling camera interface..."
raspi-config nonint do_camera 0

echo "🌐 Step 8: Enabling SSH..."
raspi-config nonint do_ssh 0

echo "🔄 Step 9: Enabling UART for motor communication..."
# Enable UART
raspi-config nonint do_serial 0
# Disable serial console
raspi-config nonint do_serial_hw 1

echo "📋 Step 10: Creating robotaxi configuration..."
cat > /home/pi/robotaxi_config.txt << EOF
# WaveShare Robotaxi Configuration
# Generated on $(date)

# Network Configuration
WIFI_SSID="your_wifi_ssid"
WIFI_PASSWORD="your_wifi_password"

# Service Ports
CONTROL_PORT=5000
JUPYTER_PORT=8888

# Motor Configuration
MOTOR_BAUDRATE=115200
MOTOR_DEVICE=/dev/ttyAMA0

# Camera Configuration
CAMERA_RESOLUTION=640x480
CAMERA_FPS=30

# Auto-start Configuration
AUTO_START_ENABLED=true
EOF

echo "✅ Setup complete!"
echo ""
echo "🎯 Next steps:"
echo "1. Edit /home/pi/robotaxi_config.txt with your Wi-Fi credentials"
echo "2. Reboot the Pi: sudo reboot"
echo "3. Access the control dashboard: http://<pi_ip>:5000"
echo "4. Access JupyterLab: http://<pi_ip>:8888"
echo ""
echo "🔧 Manual configuration:"
echo "- Run 'sudo raspi-config' to configure additional settings"
echo "- Check wiring connections (UART TX→RX, RX→TX, GND)"
echo "- Test motor control through the web interface"
echo ""
echo "📚 Documentation: https://www.waveshare.com/wiki/UGV_RPi" 