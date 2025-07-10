#!/bin/bash

# WaveShare Robotaxi - Direct Motor Control Setup
# No ESP32 required - uses GPIO pins directly

set -e  # Exit on any error

echo "🚗 WaveShare Robotaxi - Direct Motor Control Setup"
echo "=================================================="
echo "No ESP32 required - using GPIO pins directly!"
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
apt install -y git python3-venv python3-pip python3-opencv python3-flask python3-serial python3-rpi.gpio

echo "📁 Step 3: Setting up project directory..."
cd /home/pi

# Create project directory
mkdir -p robotaxi_direct
cd robotaxi_direct

echo "📥 Step 4: Downloading project files..."
# Download the direct control files
curl -O https://raw.githubusercontent.com/your-repo/direct_motor_control.py
curl -O https://raw.githubusercontent.com/your-repo/app_direct_control.py

echo "📁 Step 5: Setting up Waveshare repository..."
# Clone Waveshare repo for web interface and camera
if [ ! -d "ugv_rpi" ]; then
    git clone https://github.com/waveshareteam/ugv_rpi.git
fi

cd ugv_rpi

echo "🔐 Step 6: Making scripts executable..."
chmod +x setup.sh autorun.sh

echo "⚙️  Step 7: Running Waveshare setup (for camera and web interface)..."
./setup.sh

echo "📷 Step 8: Enabling camera interface..."
raspi-config nonint do_camera 0

echo "🌐 Step 9: Enabling SSH..."
raspi-config nonint do_ssh 0

echo "🔧 Step 10: Configuring for direct motor control..."
# Update config.yaml for direct control
sed -i 's/robot_name:.*/robot_name: "Robotaxi Direct"/' config.yaml
sed -i 's/main_type:.*/main_type: 2/' config.yaml
sed -i 's/module_type:.*/module_type: 0/' config.yaml

echo "📋 Step 11: Creating robotaxi configuration..."
cat > /home/pi/robotaxi_direct_config.txt << EOF
# WaveShare Robotaxi - Direct Motor Control Configuration
# Generated on $(date)

# Network Configuration
WIFI_SSID="your_wifi_ssid"
WIFI_PASSWORD="your_wifi_password"

# Service Ports
CONTROL_PORT=5000
JUPYTER_PORT=8888

# Motor Configuration (Direct GPIO Control)
LEFT_MOTOR_FORWARD=17
LEFT_MOTOR_BACKWARD=18
LEFT_MOTOR_PWM=23
RIGHT_MOTOR_FORWARD=27
RIGHT_MOTOR_BACKWARD=22
RIGHT_MOTOR_PWM=24

# Camera Configuration
CAMERA_RESOLUTION=640x480
CAMERA_FPS=30

# Auto-start Configuration
AUTO_START_ENABLED=true

# Direct Control Features
DIRECT_MOTOR_CONTROL=true
NO_ESP32_REQUIRED=true
LINE_FOLLOWING_ENABLED=false
OBSTACLE_AVOIDANCE_ENABLED=false
QR_CHECKIN_ENABLED=false
VOICE_COMMANDS_ENABLED=false
EOF

echo "🚀 Step 12: Creating auto-start service..."
cat > /etc/systemd/system/robotaxi-direct.service << EOF
[Unit]
Description=WaveShare Robotaxi Direct Control
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robotaxi_direct
ExecStart=/home/pi/robotaxi_direct/ugv_rpi/ugv-env/bin/python /home/pi/robotaxi_direct/app_direct_control.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo "🔧 Step 13: Enabling auto-start..."
systemctl daemon-reload
systemctl enable robotaxi-direct.service

echo "📋 Step 14: Creating test script..."
cat > /home/pi/robotaxi_direct/test_motors.py << 'EOF'
#!/usr/bin/env python3
"""
Test script for direct motor control
"""
import RPi.GPIO as GPIO
import time

def test_motors():
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor pins
    LEFT_FWD = 17
    LEFT_BWD = 18
    RIGHT_FWD = 27
    RIGHT_BWD = 22
    
    # Setup pins
    GPIO.setup(LEFT_FWD, GPIO.OUT)
    GPIO.setup(LEFT_BWD, GPIO.OUT)
    GPIO.setup(RIGHT_FWD, GPIO.OUT)
    GPIO.setup(RIGHT_BWD, GPIO.OUT)
    
    try:
        print("Testing motor connections...")
        
        # Test left motor
        print("Left motor forward (2 seconds)...")
        GPIO.output(LEFT_FWD, GPIO.HIGH)
        GPIO.output(LEFT_BWD, GPIO.LOW)
        time.sleep(2)
        GPIO.output(LEFT_FWD, GPIO.LOW)
        
        time.sleep(1)
        
        # Test right motor
        print("Right motor forward (2 seconds)...")
        GPIO.output(RIGHT_FWD, GPIO.HIGH)
        GPIO.output(RIGHT_BWD, GPIO.LOW)
        time.sleep(2)
        GPIO.output(RIGHT_FWD, GPIO.LOW)
        
        print("Motor test completed successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_motors()
EOF

chmod +x /home/pi/robotaxi_direct/test_motors.py

echo "✅ Setup complete!"
echo ""
echo "🎯 Next steps:"
echo "1. Wire up your motor driver following DIRECT_WIRING_GUIDE.md"
echo "2. Test motor connections: python3 /home/pi/robotaxi_direct/test_motors.py"
echo "3. Start the robotaxi: sudo systemctl start robotaxi-direct"
echo "4. Access web interface: http://<pi_ip>:5000"
echo ""
echo "🔧 Manual commands:"
echo "- Test motors: python3 /home/pi/robotaxi_direct/test_motors.py"
echo "- Start service: sudo systemctl start robotaxi-direct"
echo "- Stop service: sudo systemctl stop robotaxi-direct"
echo "- View logs: sudo journalctl -u robotaxi-direct -f"
echo ""
echo "📚 Documentation:"
echo "- Wiring guide: DIRECT_WIRING_GUIDE.md"
echo "- Motor driver setup required before testing"
echo ""
echo "🚗 Your robotaxi is ready for direct motor control!"
echo "No ESP32 required - using GPIO pins directly!" 