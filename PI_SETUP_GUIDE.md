# WaveShare Robotaxi - Raspberry Pi Setup Guide

## Prerequisites

Before starting, ensure you have:
- ✅ Raspberry Pi 4B (4GB or 8GB recommended)
- ✅ Waveshare Robot-Chassis MP hardware assembled
- ✅ Pi Camera Module connected
- ✅ 12V battery pack connected
- ✅ Wi-Fi connectivity for remote access
- ✅ MicroSD card with Raspberry Pi OS Lite (64-bit)

## Step 1: Initial Pi Setup

### Flash Raspberry Pi OS
1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Flash **Raspberry Pi OS Lite (64-bit)** to your microSD card
3. **Important**: Enable SSH and configure Wi-Fi during flashing
4. Insert the card into your Pi and boot

### First Boot Configuration
```bash
# SSH into your Pi (replace with your Pi's IP)
ssh pi@<your_pi_ip>
# Default password: raspberry

# Update system
sudo apt update && sudo apt full-upgrade -y

# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable
```

## Step 2: Transfer Code to Pi

### Option A: Using SCP (from your Mac)
```bash
# From your Mac terminal, in the WaveShare directory
scp -r ugv_rpi pi@<your_pi_ip>:~/
```

### Option B: Using Git (on Pi)
```bash
# SSH into Pi and clone directly
ssh pi@<your_pi_ip>
git clone https://github.com/waveshareteam/ugv_rpi.git
```

## Step 3: Install Dependencies

```bash
# Navigate to the ugv_rpi directory
cd ugv_rpi

# Make setup script executable
sudo chmod +x setup.sh

# Run the installation (this will take 10-15 minutes)
sudo ./setup.sh

# The script will:
# - Configure UART for motor communication
# - Disable Bluetooth (frees up UART)
# - Install Python dependencies
# - Configure audio settings
# - Set up camera permissions
```

## Step 4: Configure Auto-Start

```bash
# Make autorun script executable
sudo chmod +x autorun.sh

# Configure auto-start (sets up cron jobs)
./autorun.sh

# This will:
# - Set up Flask app to start on boot
# - Configure JupyterLab to start on boot
# - Remove password requirements for JupyterLab
```

## Step 5: Reboot and Test

```bash
# Reboot the Pi for all changes to take effect
sudo reboot

# Wait 2-3 minutes for the Pi to fully boot
# Then test the web interface
```

## Step 6: Access Your Robotaxi

### Web Control Interface
- **URL**: `http://<your_pi_ip>:5000`
- **Features**: 
  - Live camera feed
  - Motor control (forward, backward, turn)
  - Camera controls (zoom, capture, record)
  - Computer vision modes
  - LED controls

### JupyterLab Development Environment
- **URL**: `http://<your_pi_ip>:8888`
- **Features**:
  - Python development environment
  - Access to all robotaxi code
  - Real-time testing and debugging

## Step 7: Verify Hardware Functionality

### Test Motors
1. Open web interface: `http://<your_pi_ip>:5000`
2. Use the control buttons to test:
   - Forward/Backward movement
   - Left/Right turning
   - Speed control

### Test Camera
1. Check if live video feed appears
2. Test photo capture
3. Test video recording

### Test LEDs
1. Use LED controls in web interface
2. Verify headlights and base lights respond

## Troubleshooting

### Motors Not Responding
```bash
# Check UART connections
ls -l /dev/serial*
# Should show /dev/serial0 and /dev/serial1

# Check if UART is enabled
sudo raspi-config
# Interface Options → Serial Port → Enable

# Test serial communication
sudo python3 -c "
import serial
try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    print('Serial connection successful')
    ser.close()
except Exception as e:
    print(f'Serial error: {e}')
"
```

### Camera Not Working
```bash
# Enable camera interface
sudo raspi-config
# Interface Options → Camera → Enable

# Check camera permissions
ls -l /dev/video*
# Should show /dev/video0

# Test camera
vcgencmd get_camera
# Should return: supported=1 detected=1
```

### Web Interface Not Accessible
```bash
# Check if Flask service is running
ps aux | grep app.py

# Check if port 5000 is listening
sudo netstat -tlnp | grep 5000

# Check logs
tail -f ~/ugv.log

# Restart service manually
cd ~/ugv_rpi
source ugv-env/bin/activate
python app.py
```

### JupyterLab Not Accessible
```bash
# Check if Jupyter is running
ps aux | grep jupyter

# Check if port 8888 is listening
sudo netstat -tlnp | grep 8888

# Check logs
tail -f ~/jupyter_log.log

# Start Jupyter manually
cd ~/ugv_rpi
source ugv-env/bin/activate
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
```

## Configuration Customization

### Robot Name
Edit `ugv_rpi/config.yaml`:
```yaml
base_config:
  robot_name: "My Robotaxi"  # Change this
```

### Motor Speed Limits
```yaml
args_config:
  max_speed: 1.3    # Maximum speed (0.1-2.0)
  slow_speed: 0.2   # Slow speed for precise control
```

### Camera Settings
```yaml
video:
  default_res_w: 640   # Width
  default_res_h: 480   # Height
  default_quality: 20  # JPEG quality (1-100)
```

## Safety Notes

⚠️ **Important Safety Considerations:**
- Always test in a safe, open area
- Keep hands clear of moving parts during testing
- Monitor battery levels to prevent sudden shutdowns
- Ensure emergency stop capability is available
- The robot can move unexpectedly - stay alert!

## Next Steps

Once basic functionality is working:

1. **Phase 2 Development**: Implement line following and obstacle avoidance
2. **Custom Features**: Add your own control logic in `app.py`
3. **Advanced Autonomy**: Integrate with ROS for SLAM navigation
4. **Robotaxi Features**: Add ride request system and voice commands

## Support Resources

- [Waveshare UGV Documentation](https://www.waveshare.com/wiki/UGV_RPi)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [Project Issues](https://github.com/your-repo/issues)

---

**Your robotaxi is now ready for autonomous adventures! 🚗🤖** 