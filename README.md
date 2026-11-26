# ROS 2 Robotic Arm: URDF, Gazebo, RViz2 and Real Servos

A 4-DOF arm modelled in URDF, simulated in Gazebo (`gz sim`), visualised in
RViz2, and driven on real servo motors through a microcontroller. **One
command on `/arm_controller/joint_trajectory` moves the arm in Gazebo, in RViz2
and on the hardware at the same time.**

Target: **ROS 2 Jazzy + Gazebo Harmonic, Ubuntu 24.04** (native or WSL2 with GUI support).

```
                      /arm_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
 slider_gui  ───────────────┬──────────────────────────┬─────────────────────────┐
 trajectory_publisher ──────┘                          │                         │
                                                       ▼                         ▼
                                    JointTrajectoryController      hardware_bridge (pyserial)
                                    (ros2_control, in Gazebo)               │  "A,90,45\n"
                                                       │                    ▼
                                      /joint_states    │            Arduino firmware ──PWM──► servos
                                                       ▼
                               robot_state_publisher ──► TF ──► RViz2
```

## Packages (`src/`)

| Package | Content |
|---|---|
| `arm_description` | `arm.urdf.xacro`, RViz config, `display.launch.py` (RViz only) |
| `arm_gazebo` | world SDF, `controllers.yaml` (ros2_control), `sim.launch.py` |
| `arm_control` | `trajectory_publisher` (Python publisher), `slider_gui` (Tkinter sliders) |
| `arm_hardware` | `hardware_bridge` node (ROS 2 → serial), `hardware.launch.py`, unit tests |
| `arm_bringup` | `full.launch.py` that starts everything |
| `firmware/arm_servo/` | Arduino sketch (outside `src/`, upload with the Arduino IDE) |

## The arm

Dimensions from Figure 1 (metres in the URDF, defined as xacro properties at the
top of `arm.urdf.xacro`):

| Link | Size | URDF link | Parent joint | Axis |
|---|---|---|---|---|
| Base | 8 cm (cylinder, r = 5 cm) | `base_link` | fixed to `world` | – |
| L1 | 12 cm | `l1_link` | `joint1` (base rotation) | Z |
| L2 | 15 cm | `l2_link` | `joint2` | Y |
| L3 | 9 cm | `l3_link` | `joint3` | Y |
| End effector | 6 cm (box) | `end_effector` | `joint4` | Y |

* All 4 joints are `revolute`, limited to ±90° (±1.5708 rad) so that they map
  1:1 to a 0–180° servo (0 rad ↔ 90° servo centre).
* Every link has `<visual>`, `<collision>` and `<inertial>` (inertia computed
  from the cylinder/box formulas in the xacro macros).
* The empty `world` link and the fixed `world_to_base` joint are only the
  standard anchor that bolts the base to the ground in Gazebo; they are not a
  physical part of the arm.
* Zero pose = arm straight up. If your Figure 1 shows a different rest pose,
  only the joint angles change, not the link sizes.

## Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  python3-serial python3-tk
sudo usermod -aG dialout $USER     # serial access; log out and in afterwards
```

## Build

```bash
cd robot_arm_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional, pulls missing deps
colcon build --symlink-install
source install/setup.bash
```

## Run

### 1. URDF only, in RViz2

```bash
ros2 launch arm_description display.launch.py
```

### 2. Simulation (Gazebo + RViz2)

```bash
ros2 launch arm_gazebo sim.launch.py
```

In a second terminal (remember `source install/setup.bash`):

```bash
ros2 run arm_control trajectory_publisher          # demo motion, loops forever
# or one command from the CLI:
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2], points: [{positions: [1.0, 0.5], time_from_start: {sec: 2}}]}"
```

The arm moves in Gazebo **and** RViz2: the controller moves the Gazebo joints,
`joint_state_broadcaster` publishes them on `/joint_states`, and
`robot_state_publisher` turns them into TF for RViz2.

### 3. Full pipeline (simulation + real servos + GUI)

```bash
ros2 launch arm_bringup full.launch.py port:=/dev/ttyUSB0
```

Useful switches: `hardware:=false` (simulation only), `dry_run:=true`
(bridge prints the serial lines instead of opening a port), `gui:=false`,
`rviz:=false`.

Hardware bridge alone: `ros2 launch arm_hardware hardware.launch.py port:=/dev/ttyACM0`

## Hardware

### Parts
* 1 × ESP32 dev board (the same firmware also builds for ESP8266 and Arduino Uno/Nano)
* 2 × servos: MG996R (joint1, base rotation) and MG995 (joint2)
* **External 5–6 V supply, at least 3 A** (e.g. a 5 V/6 V 3–5 A adapter or a
  battery + buck converter). MG996R/MG995 draw up to ~2.5 A each when stalled,
  so USB or the board's 5 V pin can **not** power them.
* 470–1000 µF capacitor across the servo supply (recommended)
* Jumper wires

### Wiring

| Board pin (ESP32 / ESP8266 / Uno) | Connects to |
|---|---|
| **GPIO18 / D1 / D9** | Servo 1 signal (orange/yellow), MG996R, driven by `joint1` (base rotation) |
| **GPIO19 / D2 / D10** | Servo 2 signal (orange/yellow), MG995, driven by `joint2` (arm joint) |
| GND | Servo GND (brown/black) **and** external supply (–): *common ground* |
| – | Servo VCC (red) → external 5–6 V supply (+) (**never** the board's 5 V/VIN pin) |

Power the servos only from the external supply. A stalled MG996R/MG995 can
brown-out the board or damage the USB port. Connect the supply's (–) to the
board's GND so the signal has a common reference. If a servo jitters on the
ESP32's 3.3 V signal, add a 1 kΩ series resistor or a level shifter.

### Firmware

1. **ESP32 only:** in the Arduino IDE install the *esp32 by Espressif Systems*
   board package (Boards Manager) and the **ESP32Servo** library (Library Manager).
2. Open `firmware/arm_servo/arm_servo.ino`, select your board (e.g. *ESP32 Dev Module*)
   and port, then **Upload**. (Some ESP32 boards need the BOOT button held while
   "Connecting..." shows.)
3. The board prints `READY` at 115200 baud and centres both servos (90°).
   Quick check without ROS: open the Serial Monitor (115200, newline) and send `A,45,135`.

### Using WSL2: make the USB board visible

WSL2 does not see USB devices by default. Upload the firmware from Windows
first, then in an **administrator PowerShell**:

```powershell
winget install usbipd
usbipd list                          # find the BUSID of the ESP (CP210x / CH340)
usbipd bind --busid <BUSID>          # once
usbipd attach --wsl --busid <BUSID>  # every time you plug the board in
```

Then in Ubuntu: `ls /dev/ttyUSB* /dev/ttyACM*` and pass that port with `port:=...`.
Close the Arduino Serial Monitor first: only one program can open the port.

### Transport and message format

* **Transport:** USB serial (pyserial), 115200 baud, 8N1.
* **Message format:** one ASCII line per command, integer degrees:

  ```
  A,<servo1_deg>,<servo2_deg>\n        e.g.   A,90,45\n
  ```

  * Servo *i* ↔ `joint_names[i]` in `src/arm_hardware/config/hardware.yaml`
    (default: servo 1 = `joint1`, servo 2 = `joint2`).
  * The bridge converts `deg = offset + direction × degrees(rad)` and clamps it to
    `[min_deg, max_deg]` (default 0–180). Offsets/directions/limits are parameters.
  * The firmware clamps again to 0–180 and replies `ERR` on a malformed line.
* **Smoothness:** the bridge interpolates between trajectory points at 50 Hz using
  `time_from_start`, and only sends a line when a rounded angle changes.

## Test the full pipeline (checklist demo)

1. `ros2 launch arm_bringup full.launch.py port:=/dev/ttyUSB0`
2. Move a slider in the GUI (or run `ros2 run arm_control trajectory_publisher`).
3. Check that the arm moves in Gazebo, in RViz2, and that the two servos follow.

Handy checks:

```bash
ros2 topic echo /arm_controller/joint_trajectory
ros2 topic echo /joint_states
ros2 control list_controllers
```

## Unit tests (no ROS or hardware needed)

```bash
python3 src/arm_hardware/test/test_servo_logic.py
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Cannot open /dev/ttyUSB0` | Check `ls /dev/ttyUSB* /dev/ttyACM*`, set `port:=`, be in the `dialout` group |
| Servos jitter / board resets | Use an external supply with common ground |
| A servo moves the wrong way | Set `directions: [-1.0, 1.0]` in `hardware.yaml` |
| Arm sags/oscillates in Gazebo | Increase joint `damping` in the xacro or reduce link masses |
| Controllers not found | `ros2 control list_controllers`; make sure `gz_ros2_control` is installed |
| RViz model stays at zero pose | Check `/joint_states` is published and `use_sim_time` is true in sim |
| No window in WSL2 | Use WSLg (Windows 11) or an X server |
