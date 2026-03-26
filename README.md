# ros2_arctos

Public export of the Arctos ROS 2 workspace source tree.

Repository layout:

- `ros2/`
  - `arctos_hardware`
  - `arctos_moveit_config`
  - `arctos_urdf_description`
  - supporting ROS 2 control / topic-based hardware interface dependencies

This repository intentionally contains source packages only. It does not include
workspace `build/`, `install/`, or `log/` outputs.

Typical setup:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MATBckh22/ros2_arctos.git
rsync -a ros2_arctos/ros2/ ./
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
