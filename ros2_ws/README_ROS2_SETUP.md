# ROS 2 Workspace for WaveShare Robotaxi

This workspace contains minimal ROS 2 packages that bridge the existing
`ugv_rpi` Python chassis code to the ROS 2 Navigation stack.

## Directory structure
```
ros2_ws/
  src/
    waveshare_base/      # /cmd_vel → serial chassis driver (this repo)
    waveshare_safety/    # ultrasonic collision prevention filter
    robotaxi_launch/     # launch file that starts both nodes
```

## Quick-start on Raspberry Pi
```bash
# 1. Install ROS 2 Humble (Ubuntu 22.04) – follow tutorials
# 2. Clone your whole WaveShare repository (this repo) to ~/WaveShare
# 3. Build the workspace
cd ~/WaveShare/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 4. Run the robotaxi launch file
source install/setup.bash
ros2 launch robotaxi_launch robotaxi_launch.py
```

The launch file starts:
* `ultrasonic_safety_node`   – subscribes `/cmd_vel`, publishes `/cmd_vel_safe`
* `waveshare_base`           – drives the chassis, listens `/cmd_vel_safe`, publishes `/odom`

Now you can run Navigation2, SLAM-Toolbox or any ROS 2 tele-op tool that
publishes `/cmd_vel` and the safety node will ensure collision avoidance.

## Next steps
1. Integrate SLAM-Toolbox for mapping (publish `/map` & `/odom`).
2. Bring up `nav2_bringup` to enable global/local planning.
3. Add traffic-sign detector node (YOLO) publishing custom topics.
4. Extend the Flask web UI with ROS 2 `rosbridge_suite` if desired. 