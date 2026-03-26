# Arctos Hardware Package - Installation & Usage Guide

## Prerequisites

Before using the arctos_hardware package, install the required Python dependencies:

```bash
# Install python-can (required for CAN bus communication)
sudo apt-get install python3-can

# Or using pip (if available)
pip3 install python-can
```

## Package Contents

The `arctos_hardware` package provides complete hardware control for the Arctos robot arm:

### Python Modules

| Module | Description |
|--------|-------------|
| `can_interface.py` | Low-level CAN bus communication via SLCAN |
| `mks_servo.py` | MKS SERVO motor driver protocol implementation |
| `differential_wrist.py` | J5/J6 coupled axis kinematics |
| `homing.py` | Motor homing and calibration procedures |
| `arctos_controller.py` | High-level robot controller integrating all components |

### ROS2 Nodes

| Node | Description |
|------|-------------|
| `arctos_hardware_node.py` | Main hardware control node with services and action server |
| `arctos_hardware_interface.py` | ros2_control style hardware interface |

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `arctos_hardware.launch.py` | Launch hardware node standalone |
| `arctos_bringup.launch.py` | Full system with MoveIt2 integration |

### Configuration Files

| Config File | Description |
|-------------|-------------|
| `arctos_hardware.yaml` | Hardware parameters (CAN, motors, homing) |
| `controllers.yaml` | ros2_control controller configuration |

## Building

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arctos_hardware --symlink-install
source install/setup.bash
```

## Usage

### Launch Hardware Node

```bash
# Basic launch
ros2 launch arctos_hardware arctos_hardware.launch.py

# With an explicit serial slcan device override
ros2 launch arctos_hardware arctos_hardware.launch.py can_device:=/dev/ttyACM1
```

### Launch Full System

```bash
# Hardware + MoveIt2 + RViz
ros2 launch arctos_hardware arctos_bringup.launch.py

# With Isaac Sim
ros2 launch arctos_hardware arctos_bringup.launch.py use_isaac_sim:=true
```

### ROS2 Services

```bash
# Home all joints
ros2 service call /arctos/home_all std_srvs/srv/Trigger

# Set current position as zero
ros2 service call /arctos/set_zero std_srvs/srv/Trigger

# Enable motors
ros2 service call /arctos/enable_motors std_srvs/srv/Trigger

# Disable motors
ros2 service call /arctos/disable_motors std_srvs/srv/Trigger

# Emergency stop
ros2 service call /arctos/emergency_stop std_srvs/srv/Trigger
```

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_states` | `sensor_msgs/JointState` | Published |
| `/arctos/controller_state` | `control_msgs/JointTrajectoryControllerState` | Published |
| `/arctos/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Subscribed |

### Action Server

```bash
# Send trajectory goal
ros2 action send_goal /arctos/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 2, nanosec: 0}
"
```

## Python API

```python
from arctos_hardware import ArctosController, ArctosConfig

# Create configuration
config = ArctosConfig(
    can_device='can0',
    can_bitrate=500000,
    coupled_axis_mode=True  # Enable differential wrist
)

# Initialize controller
controller = ArctosController(config)

# Connect and enable
controller.connect()
controller.enable_motors()

# Home all joints
controller.home_all()

# Move to positions (radians)
positions = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
controller.move_to_positions(positions)

# Wait for motion complete
controller.wait_for_idle(timeout=10.0)

# Read current state
current_pos, current_vel = controller.get_joint_states()
print(f"Positions: {current_pos}")
print(f"Velocities: {current_vel}")

# Cleanup
controller.disable_motors()
controller.disconnect()
```

## Hardware Setup

### CAN Bus Connection

1. Connect the MKS SERVO CAN adapter via USB
2. Create the Linux `can0` interface:
   ```bash
   sudo ./scripts/setup_canable.sh
   ```
3. If auto-detection fails, pass the serial device explicitly:
   ```bash
   sudo ./scripts/setup_canable.sh /dev/ttyACM0
   ```
4. Ensure proper permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

### Motor CAN IDs

| Motor | CAN ID | Type | Joint |
|-------|--------|------|-------|
| 1 | 0x01 | SERVO57D | Joint 1 (Base) |
| 2 | 0x02 | SERVO57D | Joint 2 (Shoulder) |
| 3 | 0x03 | SERVO57D | Joint 3 (Elbow) |
| 4 | 0x04 | SERVO57D | Joint 4 (Wrist 1) |
| 5 | 0x05 | SERVO42D | Motor B (Wrist differential) |
| 6 | 0x06 | SERVO42D | Motor C (Wrist differential) |

### Differential Wrist

The wrist uses a differential mechanism:
- Motor B controls: J5 + J6
- Motor C controls: J5 - J6

Kinematics:
```
J5 = (Motor_B + Motor_C) / 2
J6 = (Motor_B - Motor_C) / 2
```

## Troubleshooting

### Cannot connect to CAN device

```bash
# Check CAN interface
ip -details link show can0

# Or, if using a direct serial override, check the adapter path
ls -la /dev/ttyACM*

# Bring up can0 from the adapter again
sudo ./scripts/setup_canable.sh
```

### Motors not responding

```bash
# Check CAN traffic
candump can0  # Requires can-utils

# Verify motor CAN IDs are correct
# Each motor should have unique ID 1-6
```

### Homing fails

1. Ensure mechanical limits are properly set
2. Check homing direction in configuration
3. Verify limit switch connections
4. Try homing individual joints first

### Import errors

```bash
# Rebuild package
cd ~/ros2_ws
colcon build --packages-select arctos_hardware --cmake-clean-cache
source install/setup.bash
```
