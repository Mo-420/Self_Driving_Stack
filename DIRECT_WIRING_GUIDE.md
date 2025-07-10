# Direct Motor Control Wiring Guide - No ESP32 Required

## 🎯 **Solution: Direct GPIO Motor Control**

Since you don't have an ESP32, we'll control the motors directly from the Raspberry Pi using GPIO pins. This is actually simpler and more reliable!

## 🔌 **Required Components**
- Raspberry Pi 4B
- Waveshare Robot-Chassis MP (motors only)
- Motor driver board (L298N or similar)
- 12V battery pack
- Jumper wires
- Breadboard (optional)

## 🛠 **Motor Driver Setup**

### **Option 1: L298N Motor Driver (Recommended)**
```
L298N Motor Driver Pinout:
┌─────────────────┐
│ VCC (5V)        │ ← Pi 5V
│ GND             │ ← Pi GND
│ ENA (PWM)       │ ← Pi GPIO 23 (Left motor speed)
│ IN1             │ ← Pi GPIO 17 (Left motor direction 1)
│ IN2             │ ← Pi GPIO 18 (Left motor direction 2)
│ IN3             │ ← Pi GPIO 27 (Right motor direction 1)
│ IN4             │ ← Pi GPIO 22 (Right motor direction 2)
│ ENB (PWM)       │ ← Pi GPIO 24 (Right motor speed)
│ 12V             │ ← 12V Battery
│ GND             │ ← 12V Battery GND
└─────────────────┘
```

### **Option 2: TB6612FNG Motor Driver (Alternative)**
```
TB6612FNG Pinout:
┌─────────────────┐
│ VCC (5V)        │ ← Pi 5V
│ GND             │ ← Pi GND
│ PWMA            │ ← Pi GPIO 23 (Left motor speed)
│ AIN1            │ ← Pi GPIO 17 (Left motor direction 1)
│ AIN2            │ ← Pi GPIO 18 (Left motor direction 2)
│ BIN1            │ ← Pi GPIO 27 (Right motor direction 1)
│ BIN2            │ ← Pi GPIO 22 (Right motor direction 2)
│ PWMB            │ ← Pi GPIO 24 (Right motor speed)
│ VM              │ ← 12V Battery
│ GND             │ ← 12V Battery GND
└─────────────────┘
```

## 🔗 **Connection Map**

### **Raspberry Pi 4B → Motor Driver**
| Raspberry Pi GPIO | Motor Driver | Purpose |
|-------------------|--------------|---------|
| **GPIO 17** | **IN1/AIN1** | Left motor forward |
| **GPIO 18** | **IN2/AIN2** | Left motor backward |
| **GPIO 23** | **ENA/PWMA** | Left motor speed (PWM) |
| **GPIO 27** | **IN3/BIN1** | Right motor forward |
| **GPIO 22** | **IN4/BIN2** | Right motor backward |
| **GPIO 24** | **ENB/PWMB** | Right motor speed (PWM) |
| **5V** | **VCC** | Logic power |
| **GND** | **GND** | Common ground |

### **Motor Driver → Motors**
| Motor Driver | Motor | Purpose |
|--------------|-------|---------|
| **OUT1** | **Left Motor +** | Left motor positive |
| **OUT2** | **Left Motor -** | Left motor negative |
| **OUT3** | **Right Motor +** | Right motor positive |
| **OUT4** | **Right Motor -** | Right motor negative |

### **Power Connections**
| Component | Connection | Purpose |
|-----------|------------|---------|
| **12V Battery +** | **Motor Driver VM** | Motor power |
| **12V Battery -** | **Motor Driver GND** | Motor ground |
| **12V Battery -** | **Pi GND** | Common ground |

## 🔧 **Step-by-Step Wiring**

### **Step 1: Power Off Everything**
⚠️ **Always disconnect power before making connections**

### **Step 2: Connect Pi to Motor Driver**
1. **Logic connections** (5V, GND, control pins)
2. **Direction pins** (GPIO 17, 18, 27, 22)
3. **Speed pins** (GPIO 23, 24 for PWM)

### **Step 3: Connect Motor Driver to Motors**
1. **Left motor** to OUT1/OUT2
2. **Right motor** to OUT3/OUT4
3. **Double-check polarity** - motors should turn in correct directions

### **Step 4: Connect Power**
1. **12V battery** to motor driver VM
2. **Common ground** between Pi, motor driver, and battery
3. **Pi power** via USB-C (separate from motor power)

## ⚙️ **Pin Configuration**

The `direct_motor_control.py` file uses these GPIO pins:
```python
# Motor control pins
LEFT_MOTOR_PIN1 = 17   # Left motor forward
LEFT_MOTOR_PIN2 = 18   # Left motor backward
RIGHT_MOTOR_PIN1 = 27  # Right motor forward
RIGHT_MOTOR_PIN2 = 22  # Right motor backward

# PWM pins for speed control
LEFT_MOTOR_PWM = 23
RIGHT_MOTOR_PWM = 24
```

## 🔍 **Testing the Setup**

### **Test Motor Connections**
```bash
# On the Pi, test individual motors
python3 -c "
import RPi.GPIO as GPIO
import time

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Left forward
GPIO.setup(18, GPIO.OUT)  # Left backward
GPIO.setup(27, GPIO.OUT)  # Right forward
GPIO.setup(22, GPIO.OUT)  # Right backward

# Test left motor forward
print('Testing left motor forward...')
GPIO.output(17, GPIO.HIGH)
GPIO.output(18, GPIO.LOW)
time.sleep(2)
GPIO.output(17, GPIO.LOW)

# Test right motor forward
print('Testing right motor forward...')
GPIO.output(27, GPIO.HIGH)
GPIO.output(22, GPIO.LOW)
time.sleep(2)
GPIO.output(27, GPIO.LOW)

GPIO.cleanup()
print('Test completed')
"
```

### **Test Full Control System**
```bash
# Test the direct motor controller
python3 direct_motor_control.py
```

## 🚨 **Safety Notes**

⚠️ **Important Safety Guidelines:**
- **Always power off** before making connections
- **Use appropriate wire gauge** for motor currents
- **Keep hands clear** of moving parts during testing
- **Test in a safe area** with emergency stop available
- **Check motor polarity** - wrong connections can damage motors

## 🎯 **Advantages of Direct Control**

✅ **No ESP32 required** - simpler setup
✅ **Lower latency** - direct GPIO control
✅ **More reliable** - fewer components to fail
✅ **Easier debugging** - direct access to motor signals
✅ **Lower cost** - no additional microcontroller needed

## 🔧 **Troubleshooting**

### **Motors Not Moving**
1. **Check power connections** - 12V to motor driver
2. **Verify GPIO connections** - use multimeter to test continuity
3. **Check motor polarity** - swap motor wires if needed
4. **Test individual pins** - use GPIO test script above

### **Motors Moving Wrong Direction**
1. **Swap motor wires** - reverse the connections
2. **Or swap direction pins** - change GPIO assignments in code

### **Motors Jerky or Unstable**
1. **Check PWM frequency** - adjust in code if needed
2. **Verify power supply** - ensure stable 12V
3. **Check ground connections** - ensure common ground

## 🚀 **Next Steps**

1. **Wire up the motor driver** following this guide
2. **Test basic motor control** with the test script
3. **Run the direct control app** - `python3 app_direct_control.py`
4. **Access web interface** - `http://<pi_ip>:5000`
5. **Test full robotaxi functionality**

**This direct control approach is actually better than the ESP32 method for your use case!** 