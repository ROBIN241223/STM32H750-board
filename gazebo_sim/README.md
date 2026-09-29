# STM32H750 quadcopter simulation

This package provides a ROS 2 Jazzy and Gazebo Harmonic MVP for the STM32H750 project.
It reuses the existing motor command topic without requiring an STM32 board or a serial port.

## Intended environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`
- `ros-jazzy-actuator-msgs`
- `ros-jazzy-nav-msgs`

Install the ROS dependencies if they are not already present:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-actuator-msgs ros-jazzy-nav-msgs ros-jazzy-rosgraph-msgs
```

## Build

Source ROS 2 and build this package from a colcon workspace:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/stm32_h750_ws/src
ln -sfn /home/duc/Documents/STM32H750-board/gazebo_sim \
  ~/stm32_h750_ws/src/stm32_h750_gazebo_sim
cd ~/stm32_h750_ws
colcon build --packages-select stm32_h750_gazebo_sim
source install/setup.bash
```

## Run

With ROS 2 Jazzy installed, run the complete ROS 2 simulation:

```bash
ros2 launch stm32_h750_gazebo_sim quadcopter_sim.launch.py
```

To validate only the Gazebo world without ROS 2, use the project wrapper. It sets the
local model path automatically, and the server runs headless:

```bash
bash /home/duc/Documents/STM32H750-board/gazebo_sim/run_gazebo.sh
```

To watch it, use the GUI wrapper instead. It also checks that the loaded NVIDIA kernel
module matches the userspace version before opening a window:

```bash
bash /home/duc/Documents/STM32H750-board/gazebo_sim/run_gazebo_gui.sh
```

The wrappers set `GZ_IP=127.0.0.1` and a shared `GZ_PARTITION` (`stm32_h750_sim`) so
the local GUI can discover the local server even when multicast discovery is restricted
by the network. The controller and any `gz topic` command must use the same partition:

```bash
export GZ_IP=127.0.0.1 GZ_PARTITION=stm32_h750_sim
```

The equivalent direct command is:

```bash
GZ_SIM_RESOURCE_PATH=/home/duc/Documents/STM32H750-board/gazebo_sim/models \
  gz sim --force-version 10 -r \
  /home/duc/Documents/STM32H750-board/gazebo_sim/worlds/test.world.sdf
```

Do not use `-s` when a window is wanted. `-s` is server-only mode and it **overrides
`-g`**, so `gz sim -s -g` opens no GUI at all; the GUI wrappers omit `-s` and let the
server and GUI client share one process. `--iterations 200` intentionally exits after
200 simulation iterations. The `model://x500` URI is resolved from the local model
directory and does not require Fuel.

The launch starts Gazebo, the ROS-Gazebo bridge, and the motor adapter. It does not start
`stm32_bridge`, because that node requires a real serial device. Note that the launch path
still targets the old `quadcopter` topic names — see "Known issue" at the end of this file.

## 3D world and drone model

The default world is `worlds/test.world.sdf`. It contains:

- A ground plane and a green landing pad at the origin.
- A tower obstacle at `(3, 0)`.
- Two wall obstacles at `(-3, 0)` and `(0, 4)`.
- An x500 quadrotor spawned at `(0, 0, 1)` from `models/x500/model.sdf` + `models/x500_base/model.sdf`.

The world is a 3D Gazebo environment. It is not a 2D occupancy map for navigation or SLAM.

## Direct Gazebo motor test

When running the standalone world with `run_gazebo.sh`, publish directly to the Gazebo actuator topic.
The CLI option order below is intentional, and `-d 10` keeps the publisher alive long enough for
transport discovery and delivery:

```bash
GZ_IP=127.0.0.1 GZ_PARTITION=stm32_h750_sim \
  gz topic -p 'velocity: [600, 600, 600, 600]' \
  -t /x500/command/motor_speed \
  -m gz.msgs.Actuators -d 10
```

Use `[800, 800, 800, 800]` for a short lift test and `[0, 0, 0, 0]` to stop. The model has no
attitude controller, so unequal forces and uncontrolled drift are expected.

Confirm the live topic list with `gz topic -l` before publishing; the topic name is derived
from the spawned model name, not from the world name (`quadcopter_world`).

## Motor command interface

The adapter subscribes to the existing project topic:

```text
/stm32/cmd/motor  std_msgs/msg/Float32MultiArray
[m1, m2, m3, m4, armed]
```

Each motor value is clamped to `0..255` and mapped to `0..800 rad/s`. The adapter publishes
zero rotor speed when `armed` is false or when no command is received for 250 ms.

Example command:

```bash
ros2 topic pub --once /stm32/cmd/motor std_msgs/msg/Float32MultiArray \
  "{data: [190.0, 190.0, 190.0, 190.0, 1.0]}"
```

Stop the rotors explicitly:

```bash
ros2 topic pub --once /stm32/cmd/motor std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## Simulated topics

- `/x500/imu` as `sensor_msgs/msg/Imu` (from `imu_plugin/`, ~1 kHz, body frame)
- `/x500/mag` as `sensor_msgs/msg/MagneticField`
- `/x500/command/motor_speed` as `actuator_msgs/msg/Actuators`
- `/world/quadcopter_world/pose/info` as ground-truth pose

The ground-truth pose is fed to the C estimator through `FC_Estimator_FeedPosition`, but
**only** to answer one question: *is the vehicle moving right now?* That is what the
quiescence test uses the horizontal speed for, and it is the same job PX4's land detector
does. Attitude itself is never taken from the pose — roll and pitch are gyro-propagated and
heading comes from the magnetometer. A real vehicle would answer that question from GPS
instead, which is why the position loop still needs `GPS_FUSE` and a real EKF before it
can fly off the bench.

Inspect them with:

```bash
ros2 topic echo /sim/imu
ros2 topic echo /x500/command/motor_speed
```

## Known issue: the ROS 2 launch path still targets the old model

`ros2 launch stm32_h750_gazebo_sim quadcopter_sim.launch.py` starts the motor adapter and the
ROS-Gazebo bridge, but both still use the pre-x500 names:

- `gazebo_sim/motor_adapter.py` publishes `/quadcopter/command/motor_speed`
- `config/ros_gz_bridge.yaml` bridges `/quadcopter/command/motor_speed`, `/quadcopter/imu`,
  `/quadcopter/odometry`

The world spawns `model://x500`, so those topics have no subscriber and the adapter cannot
drive the model. The verified path today is `run_gazebo.sh` / `run_gazebo_gui.sh` plus
`controller/px4_x500.py`, which uses the `/x500/*` topics. Fixing the launch path means
renaming those four topic strings; see PROJECT_LOG.md.

## Scope and limitations

This is a plant simulation, not an instruction-level STM32 simulation. It uses generic mass,
geometry and motor constants because the project does not yet provide verified quadcopter
mechanical parameters.

It is, however, firmware-in-the-loop for the control law: `controller/px4_x500.py` builds the
same `Core/Src/fc_*.c` sources into a shared library and calls them through `ctypes`, so the
estimator, attitude/rate controllers, mixer, land detector, guard and mission navigator under
test are the firmware sources, not a Python re-implementation. The Python side only supplies
sensors, ground-truth pose for logging, and the failsafe actions the firmware cannot perform
itself (teleport back to the pad).

Instruction-level simulation of the MCU (Renode) is a separate, later phase.
