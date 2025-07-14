# Robotaxi Software Quick Reference

This one-pager gets a fresh developer workstation from zero to running and testing the WaveShare Robotaxi software stack.

---

## 1. Clone the repo
```bash
git clone https://github.com/<your_user>/WaveShare.git
cd WaveShare
```

## 2. Install ROS 2 (Ubuntu 22.04/Humble)
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
source /etc/os-release   # sets $UBUNTU_CODENAME → jammy
sudo sh -c "echo \"deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main\" > /etc/apt/sources.list.d/ros2.list"

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update

# Permanently source ROS in your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Build the workspace
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. Launch the stack
### Simulation (no real hardware)
```bash
ros2 launch waveshare_base robotaxi_bringup.launch.py use_sim_time:=true
```

### Real hardware (Raspberry Pi)
```bash
ros2 launch waveshare_base robotaxi_bringup.launch.py
```

## 5. Run automated tests
```bash
colcon test && colcon test-result --verbose
```

## 6. Code quality checks
```bash
black .            # auto-format
flake8 .           # lint
mypy ros2_ws/src   # type-check
```
(The git *pre-commit* hook supplied in `githooks/pre-commit` runs the same trio automatically.)

## 7. Common pitfalls
* **Environment not sourced** → `source install/setup.bash` in every new shell.
* **Serial permission errors** → add user to `dialout` & `gpio` groups: `sudo usermod -a -G dialout,gpio $USER`.
* **Missing build deps** → re-run `rosdep install` after pulling new packages.

## 8. Continuous Integration
Every push & pull-request is built and tested by GitHub Actions (workflow file: `.github/workflows/ci.yml`).

---
For a deeper dive see `README.md`, `PI_SETUP_GUIDE.md`, and the in-code doc-strings. Happy hacking! 🚀