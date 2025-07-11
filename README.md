# WaveShare Robotaxi Project

[![ROS 2 CI](https://github.com/Mo-420/Self_Driving_Stack/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/Mo-420/Self_Driving_Stack/actions/workflows/ros2_ci.yml)

A ROS 2-based autonomous robotaxi system built on the WaveShare Robot-Chassis MP platform.

## 🏗️ Architecture

```
┌─────────────────────┐         ┌──────────────────────────────────┐
│   Web Application   │         │        Raspberry Pi 4B           │
│  ┌───────────────┐  │         │  ┌────────────────────────────┐  │
│  │ Flask Server  │  │ Socket  │  │      ROS 2 Stack           │  │
│  │ + Socket.IO   │◄─┼────────►│  │  ┌──────────┐ ┌────────┐  │  │
│  └───────────────┘  │   IO    │  │  │Navigation│ │Safety  │  │  │
│         ▲           │         │  │  └─────┬────┘ └───┬────┘  │  │
│         │           │         │  │        │          │        │  │
│    Route Planning   │         │  │  ┌─────▼──────────▼─────┐  │  │
│         │           │         │  │  │    Motor Control     │  │  │
│         ▼           │         │  │  └──────────┬───────────┘  │  │
│   ┌───────────┐     │         │  └─────────────┼──────────────┘  │
│   │ Database  │     │         │                │ UART            │
│   └───────────┘     │         │         ┌──────▼──────┐          │
└─────────────────────┘         │         │   ESP32     │          │
                                │         │ Controller  │          │
                                │         └─────────────┘          │
                                └──────────────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites
- Raspberry Pi 4B with Ubuntu 22.04 or Raspberry Pi OS
- WaveShare Robot-Chassis MP
- Pi Camera Module (optional)
- 12V battery pack

### Installation

```bash
# Clone repository
git clone https://github.com/Mo-420/Self_Driving_Stack.git WaveShare
cd WaveShare

# Run automated setup
sudo ./setup_robotaxi.sh

# Install ROS 2 (if not already installed)
bash setup_pi_robotaxi.sh

# Build ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Test hardware connectivity
python3 ../test_waveshare_hardware.py
```

### Launch the Robotaxi

```bash
# Single command to start everything
ros2 launch waveshare_base robotaxi_bringup.launch.py

# Or with web server integration
ros2 launch waveshare_base robotaxi_bringup.launch.py \
    web_server_url:=http://192.168.1.100:5000
```

## 📦 ROS 2 Packages

### waveshare_base
Core hardware interface and control
- **base_node** - Motor control via UART/JSON protocol
- **encoder_odom_node** - Wheel encoder odometry

### waveshare_perception  
Computer vision and environment understanding
- **yolo_sign_node** - Traffic sign detection
- **yolo_traffic_light_node** - Traffic light detection
- **lane_segmentation_node** - Lane boundary detection
- **object_detection_node** - General obstacle detection

### waveshare_safety
Safety-critical systems
- **ultrasonic_safety_node** - Emergency braking from ultrasonic sensors

### waveshare_navigation
Autonomous navigation and path planning
- **goal_follower_node** - Waypoint navigation
- **rules_of_road_node** - Traffic rules compliance
- **route_receiver_node** - Web app route integration
- **telemetry_sender_node** - Real-time status updates

## 🔌 Hardware Setup

### Wiring Connections
```
Raspberry Pi 4B          WaveShare ESP32
─────────────────       ─────────────────
GPIO 14 (TX)  ────────► RX
GPIO 15 (RX)  ◄──────── TX
GND           ────────  GND
```

### Power Configuration
- 12V Battery → Chassis Power Input
- 5V USB-C → Raspberry Pi Power

### Sensor Connections
- Pi Camera → CSI Port
- Ultrasonic Sensors → GPIO pins (configurable)

## 🌐 Web Integration

The robotaxi integrates with a web application for:
- Route planning and dispatch
- Real-time telemetry monitoring
- Remote emergency stop
- Fleet management

### API Endpoints
- `POST /api/route` - Submit route request
- `GET /api/status/<trip_id>` - Check trip status
- WebSocket events: `route`, `cancel`, `telemetry`

## 🛠️ Development

### Building from Source
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running Tests
```bash
colcon test
colcon test-result --verbose
```

### Code Quality
```bash
# Linting
flake8 ros2_ws/src --config=.flake8

# Format code
black ros2_ws/src
```

## 🚦 Current Status

### ✅ Implemented
- Hardware abstraction layer for WaveShare chassis
- Wheel encoder odometry
- YOLO-based perception pipeline
- Basic waypoint following
- Emergency stop functionality
- Web app integration
- Telemetry feedback
- Systemd auto-start service
- GitHub Actions CI/CD

### 🚧 In Progress
- Nav2 integration for advanced path planning
- SLAM-based mapping
- Multi-robot coordination
- Battery management system
- Advanced traffic rules engine

### 📋 Planned
- Simulation environment (Gazebo)
- Hardware-in-the-loop testing
- Cloud-based fleet management
- V2X communication
- Charging station docking

## 📊 Performance Metrics

| Metric | Value |
|--------|-------|
| Max Speed | 0.3 m/s |
| Turn Radius | 0.15 m |
| Battery Life | ~2 hours |
| Perception Rate | 10 Hz |
| Control Loop | 20 Hz |
| Telemetry Update | 1 Hz |

## 🔧 Configuration

Key parameters can be adjusted in launch files:
```python
# Speed limits
max_linear_vel: 0.3  # m/s
max_angular_vel: 1.0  # rad/s

# Safety thresholds  
stop_distance: 0.3  # meters
slow_distance: 1.0  # meters

# Navigation tuning
goal_tolerance: 0.5  # meters
```

## 📚 Documentation

- [Hardware Compatibility](HARDWARE_COMPATIBILITY_SUMMARY.md)
- [Wiring Guide](WIRING_GUIDE.md) 
- [Pi Setup Guide](PI_SETUP_GUIDE.md)
- [Development Roadmap](ROADMAP.md)
- [API Reference](docs/api_reference.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- WaveShare team for the excellent Robot-Chassis MP platform
- ROS 2 community for the robust robotics middleware
- Ultralytics for YOLO object detection models 