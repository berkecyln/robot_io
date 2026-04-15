# GELLO Teleoperation with robot_io

For hardware setup, motor ID assignment, USB configuration, and GELLO calibration,
see [`gello_software/docs/gello-setup.md`](../../gello_software/docs/gello-setup.md).

This document covers only the robot_io integration.

## 1. Install Dependencies

Inside the `robotio` conda environment:

```bash
conda activate robotio
pip install -e /path/to/gello_software
pip install dynamixel-sdk
pip install tyro
```

Verify:
```bash
python -c "from gello.agents.gello_agent import GelloAgent; print('OK')"
```


## 2. Verify Hardware Connection

Test GELLO reads correctly (no robot needed):
```bash
conda activate robotio
python -c "
from robot_io.input_devices.gello_input import GelloInput
g = GelloInput()
action, _ = g.get_action()
print('arm joints:', action['motion'])
print('gripper:', action['gripper'])   # 1=open, -1=closed
g.close()
"
```
Move the arm and rerun, values should change. Squeeze trigger, gripper should flip to `-1`.

## 3. Configuration

**`robot_io/conf/gello_teleop.yaml`**  tunable parameters:
```yaml
freq: 60          # control loop Hz
robot:
  franky_params:
    velocity_rel: 0.4     # robot speed factor (0.0–1.0)
    acceleration_rel: 0.4 # increase if sluggish, decrease if too aggressive
    jerk_rel: 0.4
```

**`robot_io/conf/input/gello.yaml`** hardware parameters:
```yaml
port: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4UAS-if00-port0
gripper_close_threshold: 0.5  # 0=open, 1=closed — adjust if trigger is too sensitive
```

## 4. Teleoperation

```bash
conda activate robotio
cd /path/to/robot_io
python robot_io/examples/gello_teleop.py
```

**Startup sequence:**
1. Robot moves to neutral and runs gripper homing
2. Match GELLO arm to robot pose then press **Enter**
3. Focus cv2 window for keyboard input

**Controls:**
| Key | Action |
|-----|--------|
| `r` | Toggle recording ON/OFF |
| `q` | Quit and save trajectory |

**Output:** `trajectory_YYYYMMDD_HHMMSS.npy` in the current directory, list of dicts per frame:

| Field | Shape | Description |
|-------|-------|-------------|
| `tcp_pos` | (3,) | End-effector position (m) |
| `tcp_orn` | (4,) | Quaternion xyzw |
| `joint_positions` | (7,) | Joint angles (rad) |
| `gripper` | int | 1=open, -1=closed |
| `timestamp` | float | Unix time |

## 6. Troubleshooting

| Problem | Fix |
|---------|-----|
| `No module named 'dynamixel_sdk'` | `pip install dynamixel-sdk` |
| `No module named 'gello'` | `pip install -e /path/to/gello_software` |
| Robot jerks during teleop | Check latency timer  |
| Gripper doesn't move | Check Franka Desk robot must be in ready state; homing runs at startup |
| `RuntimeError: Motion planner failed -111` | Joint limit hit reduce `velocity_rel` in `gello_teleop.yaml` |

