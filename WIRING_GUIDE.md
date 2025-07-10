# Wiring Guide - WaveShare Robot-Chassis MP ↔ Raspberry Pi 4B

## Overview
This guide shows how to connect your Raspberry Pi 4B to the Waveshare Robot-Chassis MP for motor control and communication.

## Required Components
- Raspberry Pi 4B
- Waveshare Robot-Chassis MP
- Pi Camera Module (optional for video streaming)
- 12V battery pack
- Jumper wires (male-to-male)
- Breadboard (optional, for testing)

## Pin Connections

### Primary UART Communication
| Raspberry Pi 4B | Waveshare Chassis | Purpose |
|-----------------|-------------------|---------|
| **GPIO 14 (TX)** | **ESP32 RX** | Serial data from Pi to chassis |
| **GPIO 15 (RX)** | **ESP32 TX** | Serial data from chassis to Pi |
| **GND** | **GND** | Common ground |
| **5V** | **5V** | Power (if needed) |

### Camera Connection (Optional)
| Raspberry Pi 4B | Camera Module | Purpose |
|-----------------|---------------|---------|
| **CSI Port** | **Camera Cable** | Video data |
| **3.3V** | **3.3V** | Camera power |
| **GND** | **GND** | Camera ground |

## Step-by-Step Connection

### Step 1: Power Off Everything
⚠️ **Always disconnect power before making connections**

### Step 2: Connect UART Pins
1. Connect Pi GPIO 14 (TX) to chassis ESP32 RX pin
2. Connect Pi GPIO 15 (RX) to chassis ESP32 TX pin
3. Connect Pi GND to chassis GND
4. **Double-check**: TX→RX, RX→TX (crossed connection)

### Step 3: Connect Camera (if using)
1. Insert camera cable into Pi CSI port
2. Ensure cable is properly seated and locked

### Step 4: Power Connections
1. Connect 12V battery to chassis power input
2. Power Pi via USB-C or GPIO power pins

## Verification Steps

### Test UART Communication
```bash
# On Raspberry Pi, check if UART is enabled
ls /dev/ttyAMA0

# Test serial communication
sudo apt install minicom
sudo minicom -D /dev/ttyAMA0 -b 115200
```

### Test Camera
```bash
# Check if camera is detected
vcgencmd get_camera

# Test camera capture
raspistill -o test.jpg
```

## Troubleshooting

### Motors Not Responding
1. **Check UART connections**: TX→RX, RX→TX
2. **Verify ground connection**: Must have common GND
3. **Check baud rate**: Default is 115200
4. **Test with multimeter**: Verify continuity

### Serial Communication Issues
```bash
# Check UART status
sudo raspi-config nonint get_serial

# Enable UART if needed
sudo raspi-config nonint do_serial 0
sudo raspi-config nonint do_serial_hw 1
```

### Camera Not Working
```bash
# Enable camera interface
sudo raspi-config nonint do_camera 0

# Check camera permissions
sudo usermod -a -G video pi
```

## Safety Notes

⚠️ **Important Safety Guidelines:**
- Always power off before making connections
- Double-check all connections before powering on
- Use appropriate wire gauge for power connections
- Keep hands clear of moving parts during testing
- Have an emergency stop method available

## Advanced Configuration

### Custom UART Settings
Edit `/boot/config.txt`:
```
# Enable UART
enable_uart=1
# Set baud rate
dtoverlay=uart0,ctsrts
```

### GPIO Pin Mapping
If you need to use different pins:
```python
# In motor_control.py
SERIAL_PORT = '/dev/ttyAMA0'  # or '/dev/ttyS0'
BAUDRATE = 115200
```

## Testing Checklist

- [ ] UART communication established
- [ ] Motors respond to commands
- [ ] Camera streams video
- [ ] Web dashboard accessible
- [ ] Auto-start working after reboot
- [ ] Emergency stop functional

## Next Steps

After successful wiring:
1. Run the setup script: `sudo ./setup_robotaxi.sh`
2. Test motor control via web interface
3. Verify camera streaming
4. Configure auto-start
5. Begin Phase 2 development (autonomy features) 