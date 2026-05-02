# Hamerschlag Haul

An autonomous indoor delivery robot built on ROS2. The robot maps its environment using RPLiDAR SLAM, navigates to named delivery waypoints on request via a REST API, and monitors its surroundings with a Kria FPGA-based person detector and a motion-tracking pan servo.

---

## Hardware

| Component | Details |
|---|---|
| **Compute** | Raspberry Pi (Linux, ROS2) |
| **FPGA** | Kria SOM (person detection inference) |
| **LiDAR** | RPLiDAR A1 — `/dev/ttyUSB0` @ 115200 baud |
| **Odometry** | SparkFun Qwiic OTOS optical tracking sensor (I2C) |
| **Motors** | 2× DC motors via VNH5019 dual H-bridge |
| **Drive** | Differential drive, wheel base 0.30 m |
| **Pan Servo** | GPIO 18, 0.4–2.5 ms pulse width |

**Motor GPIO wiring:**

| Side | INA | INB | PWM | CS |
|---|---|---|---|---|
| Left | 17 | 27 | 13 | 26 |
| Right | 22 | 23 | 12 | 16 |

---

## System Architecture

```
┌──────────────────────────────────────────────────┐
│            Dispatch & Mission Control            │
│  dispatch_server.py — Flask REST API :5000       │
│  POST /dispatch {"destination":"A"} → Nav2 goal  │
└──────────────────────┬───────────────────────────┘
                       │ nav2_simple_commander
                       v
┌──────────────────────────────────────────────────┐
│              Nav2 Navigation Stack               │
│  BT Navigator + DWB Local Planner                │
│  max_vel_x=0.15 m/s  max_vel_θ=0.50 rad/s        │
└──────────────────────┬───────────────────────────┘
                       │ /cmd_vel_nav
                       v
              velocity_smoother
                       │ /cmd_vel
                       v
┌──────────────────────────────────────────────────┐
│             vnh5019_motor_run.py                 │
│  Differential-drive kinematics → PWM @ 1 kHz    │
│  Watchdog: stops if no cmd_vel for 0.5 s         │
└───────────────┬──────────────────────────────────┘
           LEFT / RIGHT motors

┌──────────────────────────────────────────────────┐
│                 Localization                     │
│  RPLiDAR A1 → /scan                              │
│  Cartographer (mapping or localization mode)     │
│  OTOS → otos_odom.py → /odom + odom→base_link   │
│  (yaw covariance=99 forces scan-match heading)   │
└──────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────┐
│                  Perception                      │
│  Kria FPGA :5555 → det_bridge.py                 │
│    Alarms via espeak (fallen / running person)   │
│  RPLiDAR /scan → pan_tilt.py                     │
│    Servo tracks motion direction                 │
└──────────────────────────────────────────────────┘
```

---

## ROS2 Nodes

### `otos_odom.py`
Reads the SparkFun OTOS over I2C and publishes `/odom` + the `odom→base_link` TF at 20 Hz.

The OTOS reports Y as forward and X as rightward in its native frame; this node remaps to standard ROS convention (`x_ros = pos.y`, `y_ros = -pos.x`). Yaw covariance is set to 99.0 deliberately — this tells Cartographer to ignore OTOS heading and rely on LiDAR scan matching instead.

### `vnh5019_motor_run.py`
Subscribes to `/cmd_vel` Twist messages and drives the two VNH5019 H-bridge channels. Implements differential-drive kinematics, normalizes to `[-1, 1]`, enforces a minimum duty cycle of 18% (stall threshold), and applies a 15% right-side trim to compensate for motor mismatch. A 0.5 s watchdog stops the motors if no command arrives.

### `det_bridge.py`
Opens a TCP socket on port 5555 and receives JSON person-detection frames from the Kria FPGA. Issues `espeak` audio alarms:
- **Fallen person** — immediate on first `laying_down=true` detection (5 s cooldown)
- **Running person** — after 6 consecutive `running=true` frames (5 s cooldown)

Publishes all detections to `/person_detections`.

### `dispatch_server.py`
Flask server (port 5000) that accepts delivery commands and drives the Nav2 stack via `nav2_simple_commander.BasicNavigator`.

**Endpoints:**

| Method | Path | Body | Description |
|---|---|---|---|
| `GET` | `/status` | — | Current state and destination |
| `POST` | `/dispatch` | `{"destination":"A"}` | Navigate to waypoint A–D |
| `POST` | `/return` | — | Return to home (0, 0) |
| `GET` | `/map` | — | Raw PGM map image |

**Waypoints** (edit in `dispatch_server.py` lines 62–68):

| Label | x (m) | y (m) | Heading |
|---|---|---|---|
| A | 1.00 | 0.50 | 0° |
| B | 2.50 | 1.00 | 90° |
| C | 2.50 | -1.00 | -90° |
| D | 0.50 | -1.50 | 180° |

### `pan_tilt.py`
Subscribes to `/scan` and compares consecutive scans. If more than 15 rays change by >0.3 m, the servo pans toward whichever half of the scan (left or right) had larger motion. The servo auto-detaches 0.6 s after each move to save power. Minimum 2 s between consecutive moves.

### `forward_stop.py`
Utility node that publishes a scripted sequence of Twist commands (forward → turn left → stop) for building a map of a new space without manual driving.

---

## Launch Files

### `master_launch.py` — full stack (mapping + localization + Nav2)
Starts: `robot_state_publisher`, `rplidar_node`, `cartographer_node` (localization mode from `map_new.pbstream`), `map_server`, and the full Nav2 lifecycle stack with velocity smoother.

### `nav2_launch.py` — Nav2 only
Starts `map_server` and Nav2 assuming localization is already provided externally. Use this when running Cartographer separately.

---

## Configuration

| File | Purpose |
|---|---|
| [config/nav2_params.yaml](config/nav2_params.yaml) | Nav2 controller, planner, costmap settings |
| [config/rplidar_a1.lua](config/rplidar_a1.lua) | Cartographer mapping parameters |
| [config/rplidar_a1_localization.lua](config/rplidar_a1_localization.lua) | Cartographer localization parameters (stricter) |

Key Nav2 tuning values: `xy_goal_tolerance=0.50 m`, `yaw_goal_tolerance=0.5 rad`, `acc_lim_x/theta=0.5`.

---

## Calibration

Run these before deploying to a new environment or after mechanical changes.

**OTOS linear and angular scalars:**
```bash
python3 otos_calibrate.py
```
Follow the prompts (push robot 1 m forward, then rotate 360°). Copy the output scalars into `otos_odom.py` (`setLinearScalar` / `setAngularScalar`).

**Angular calibration via spinning:**
```bash
python3 spin_calibration.py
```

**Motor straight-line verification:**
```bash
python3 straight_calibration.py
```
Drives forward then backward for 15 s each. Adjust `right_trim` in `vnh5019_motor_run.py` if the robot pulls to one side.

---

## Running the Robot

**1. Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select hamerschlag_haul
source install/setup.bash
```

**2. Start the full navigation stack:**
```bash
ros2 launch hamerschlag_haul master_launch.py
```

**3. Start motor control and odometry (separate terminals):**
```bash
ros2 run hamerschlag_haul vnh5019_motor_run
ros2 run hamerschlag_haul otos_odom
```

**4. Start the dispatch server:**
```bash
python3 dispatch_server.py
```

**5. Send a delivery command:**
```bash
curl -X POST http://<robot_ip>:5000/dispatch -H "Content-Type: application/json" -d '{"destination":"A"}'
curl http://<robot_ip>:5000/status
curl -X POST http://<robot_ip>:5000/return
```

---

## Building a New Map

Use Cartographer in mapping mode and drive the robot through the space:

```bash
# Launch with mapping config (rplidar_a1.lua instead of localization)
ros2 run hamerschlag_haul otos_odom
ros2 run hamerschlag_haul forward_stop   # or drive manually with teleop
```

Save the finished map:
```bash
ros2 run nav2_map_server map_saver_cli -f config/map_new
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'config/map_new.pbstream'}"
```

Update `master_launch.py` to point to the new `.pbstream` and `.yaml` files.

---

## Dependencies

```
rclpy
nav_msgs  geometry_msgs  sensor_msgs  tf2_ros
rplidar_ros
slam_toolbox
nav2_bringup  navigation2  nav2_simple_commander
cartographer_ros
rviz2
flask
sparkfun-qwiic-otos  (I2C OTOS driver)
RPi.GPIO  (motor and servo GPIO)
espeak  (audio alarms)
```

Install ROS2 dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
