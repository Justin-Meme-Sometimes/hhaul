# Hamerschlag Haul

Autonomous delivery robot built on a Raspberry Pi 5 with ROS2 Jazzy, Google Cartographer SLAM, Nav2 navigation, FPGA-accelerated person detection, and a behavioral safety protocol system. Designed for indoor autonomous delivery in hallway environments.

---

## Hardware

| Component | Details |
|---|---|
| Main computer | Raspberry Pi 5 (8GB) |
| FPGA board | AMD Kria KR260 |
| LiDAR | RPLiDAR A1M8 (USB) |
| Odometry sensor | SparkFun Qwiic OTOS (PAA5160E1, I2C) |
| Motor driver (final) | Pololu VNH5019 dual motor driver |
| Motor driver (dev) | Elegoo Smart Car V4 with Arduino UNO + TB6612FNG |
| Microcontroller | Arduino UNO (USB serial bridge to Pi) |
| Camera | USB camera with pan-tilt servo mount |

**SSH access:** `ssh hhaul@172.26.19.241`  
**Username:** `hhaul`

---

## Software Stack

### Pi 5 OS

- **Ubuntu 24.04 LTS** (Noble) — 64-bit ARM (`aarch64`)
- **ROS2 Jazzy Jalopy**
- **ROS_DOMAIN_ID=42**

### ROS2 Packages (Pi)

```bash
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-cartographer \
  ros-jazzy-cartographer-ros \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-nav2-smoother \
  ros-jazzy-rplidar-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-twist-mux \
  ros-jazzy-teleop-twist-keyboard
```

### Python Dependencies (Pi)

```bash
pip install sparkfun-qwiic-otos --break-system-packages
pip install pyserial --break-system-packages
pip install lgpio --break-system-packages
sudo apt install python3-gpiozero python3-lgpio
```

### ROS2 Setup (add to `~/.bashrc`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

---

## Workspace Layout

```
~/ros2_ws/src/hamerschlag_haul/
├── config/
│   ├── rplidar_a1.lua              # Cartographer 2D SLAM config
│   ├── rplidar_a1_localization.lua # Cartographer localization config
│   └── nav2_params.yaml            # Nav2 full stack parameters
├── launch/
│   ├── cartographer_launch.py      # SLAM mapping mode
│   ├── cartographer_localization_launch.py  # Localization mode
│   ├── nav2_launch.py              # Nav2 stack
│   └── master_launch.py            # Full system (localization + Nav2)
├── hamerschlag_haul/
│   ├── otos_odom_node.py           # SparkFun OTOS -> /odom + TF
│   ├── arduino_motor_bridge.py     # /cmd_vel -> Arduino serial
│   ├── detection_bridge_node.py    # TCP socket -> /person_detections
│   ├── protocol_manager.py         # Behavioral FSM (CLEAR/MONITOR/ALERT/EMERGENCY/LOCKDOWN)
│   ├── speed_limiter.py            # Nav2 speed limiter based on FSM state
│   ├── pan_tilt_tracker_node.py    # Servo pan-tilt based on LiDAR scan sectors
│   └── laser_tf_broadcaster.py     # Static TF: base_link -> laser_frame
├── urdf/
│   └── robot.urdf                  # Robot description
└── setup.py
```

### Map files (saved to home directory)

```
~/map.pbstream   # Cartographer state (used for localization)
~/map.pgm        # Occupancy grid image (used by Nav2 map server)
~/map.yaml       # Map metadata (used by Nav2 map server)
```

---

## TF Tree

```
map
└── odom             (published by Cartographer or AMCL)
    └── base_link    (published by otos_odom_node.py)
        └── laser_frame  (static, published by robot_state_publisher)
```

**Critical notes:**
- `published_frame = "odom"` in the Cartographer lua when using OTOS odometry — never `"base_link"` or it conflicts with the OTOS TF broadcast
- `ignore_timestamp: True` in `robot_state_publisher` parameters to prevent stale joint state timestamps from breaking TF lookups
- OTOS odometry publisher uses **BEST_EFFORT** QoS — using RELIABLE causes Cartographer to silently drop messages

---

## Google Cartographer SLAM

### Key config parameters (`rplidar_a1.lua`)

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",       -- MUST be "odom" when use_odometry = true
  odom_frame = "odom",
  provide_odom_frame = false,     -- false when OTOS provides odom
  publish_frame_projected_to_2d = true,
  use_odometry = true,            -- true when OTOS is running
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options
```

### Mapping workflow (run once to build the map)

```bash
# Terminal 1 - OTOS odometry
python3 ~/ros2_ws/src/hamerschlag_haul/hamerschlag_haul/otos_odom_node.py

# Terminal 2 - LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# Terminal 3 - SLAM
ros2 launch ~/ros2_ws/src/hamerschlag_haul/launch/cartographer_launch.py

# Terminal 4 - RViz2 to monitor the map
rviz2
```

Drive the robot around the entire environment. When satisfied:

```bash
# Save the Cartographer state
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/hhaul/map.pbstream'}"

# Save the occupancy grid for Nav2
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Verify saved files

```bash
ls ~/map*
# Should show: map.pbstream  map.pgm  map.yaml
```

---

## SparkFun OTOS Odometry

**Sensor:** SparkFun Qwiic Optical Tracking Odometry Sensor (PAA5160E1)  
**Interface:** I2C (Qwiic connector)  
**Library:** `sparkfun-qwiic-otos`

### Verify sensor is detected on I2C bus

```bash
sudo apt install i2c-tools
i2cdetect -y 1
# Try bus 0 or 4 if bus 1 shows nothing on Pi 5
```

### Test sensor standalone

```bash
pip install sparkfun-qwiic-otos --break-system-packages
git clone https://github.com/sparkfun/Qwiic_OTOS_Py.git
cd Qwiic_OTOS_Py/examples
python3 ex1_basic_readings.py
```

### What `otos_odom_node.py` does

- Reads x, y, heading from sensor (native: inches and degrees)
- Converts to meters and radians (ROS standard)
- Publishes `nav_msgs/Odometry` on `/odom` with **BEST_EFFORT** QoS
- Broadcasts `odom -> base_link` TF transform
- Uses a tight `while` loop (not a ROS timer) for maximum TF timestamp continuity

### Mounting requirements

The OTOS must be mounted **rigidly and flat** on the robot chassis. Handheld testing produces unusable maps — any tilt or vibration registers as robot movement and corrupts Cartographer's pose estimate.

---

## RPLiDAR A1

**Interface:** USB (`/dev/ttyUSB0` typically)  
**Package:** `ros-jazzy-rplidar-ros`

```bash
# Launch the LiDAR driver
ros2 launch rplidar_ros rplidar_a1_launch.py

# Verify it's publishing
ros2 topic echo /scan

# Check the USB port
ls /dev/ttyUSB*
```

**LiDAR mounting height:** 15-20cm above floor. Too low picks up under furniture and produces bad scan data. Too high misses obstacles.

---

## Nav2

### Full navigation stack (run after mapping is done)

```bash
# This launch starts: Cartographer localization + Nav2 (AMCL, map server, controller, planner, costmaps)
ros2 launch ~/ros2_ws/src/hamerschlag_haul/launch/master_launch.py
```

Wait ~10 seconds for Nav2 lifecycle manager to activate all nodes.

### Verify Nav2 is running

```bash
ros2 node list | grep nav2
ros2 topic list | grep costmap
```

### Set a navigation goal from CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

### Nav2 costmap topics (add in RViz2)

- `/global_costmap/costmap` — Map display, color scheme: costmap
- `/local_costmap/costmap` — Map display, color scheme: costmap
- `/global_costmap/published_footprint` — Polygon display
- `/local_costmap/published_footprint` — Polygon display

---

## Arduino Motor Bridge

The Arduino UNO acts as a real-time PWM controller between the Pi and the VNH5019 motor drivers. The Pi sends velocity commands over USB serial.

**Serial port:** `/dev/ttyUSB1` (113200 baud)  
**Protocol:** Text-based, `<` start marker, `\n` terminator

```
Pi -> Arduino:   <CMD,left_pwm,right_pwm,left_dir,right_dir\n
Pi -> Arduino:   <HB\n                (heartbeat, sent every 500ms)
Pi -> Arduino:   <STOP\n             (emergency stop)
Arduino -> Pi:   <ACK,left_pwm,right_pwm,left_dir,right_dir,millis\n
Arduino -> Pi:   <ERR,message\n
```

**Safety watchdog:** Arduino stops both motors if no heartbeat received within 1 second.

### VNH5019 pin mapping (Arduino UNO)

| Signal | Arduino Pin |
|---|---|
| Left PWM | 9 |
| Left INA | 4 |
| Left INB | 5 |
| Left EN | 6 |
| Right PWM | 10 |
| Right INA | 7 |
| Right INB | 8 |
| Right EN | 11 |

### Test serial link (no ROS2 needed)

```bash
python3 ~/ros2_ws/src/hamerschlag_haul/hamerschlag_haul/test_serial.py
```

### ROS2 motor bridge node

Subscribes to `/cmd_vel` (Twist from Nav2), converts linear.x and angular.z to per-wheel PWM, sends over serial. Publishes Arduino ACKs to `/motor_state` as JSON.

---

## FPGA — AMD Kria KR260

The KR260 runs YOLOX-Nano person detection via Vitis AI DPU B4096. Detection results are sent to the Pi over TCP socket.

**KR260 ROS_DOMAIN_ID:** 42 (same as Pi)

### Detection pipeline

```
USB Camera -> GStreamer -> YOLOX-Nano INT8 (DPU B4096) -> Post-processing -> TCP socket -> Pi
```

Post-processing includes:
- DPU output decoding (3 feature map scales: 52x52, 26x26, 13x13)
- Sigmoid/softmax scoring
- Score threshold filtering (class 0 = person only)
- Greedy NMS with IoU threshold 0.45 (fixed-point HLS implementation)

### Detection message format (JSON over TCP port 5555)

```json
[
  {"x1": 100, "y1": 50, "x2": 300, "y2": 400, "score": 0.87},
  ...
]
```

### On Pi side

`detection_bridge_node.py` listens on TCP port 5555, reads JSON lines from the KR260, and publishes to `/person_detections` as `std_msgs/String`.

**Start order:** Pi listener first, then KR260 sender.

```bash
# Pi
python3 ~/ros2_ws/src/hamerschlag_haul/hamerschlag_haul/detection_bridge_node.py

# KR260 (after Pi is listening)
python3 gst_normal_camera_yolox_test.py
```

---

## Behavioral Protocol FSM

The `protocol_manager.py` node subscribes to `/person_detections` and manages the robot's behavioral state machine.

### States

| State | Condition | Behavior |
|---|---|---|
| CLEAR | No person detected | Full navigation speed |
| MONITOR | Person detected, not close | Reduced speed, tracking active |
| ALERT | Person in close range | Slow speed, announce |
| EMERGENCY | Person blocking path | Stop, wait |
| LOCKDOWN | Extended blockage or failure | Full stop, alert operator |

The `speed_limiter.py` node receives the FSM state and publishes speed limits to Nav2's velocity smoother.

---

## Pan-Tilt Camera Tracker

`pan_tilt_tracker_node.py` monitors `/scan` from the LiDAR to detect motion in sectors not visible to the forward camera. When motion is detected outside the camera FOV, it commands the pan-tilt servo to snap to one of 4 preset orientations (0°, 90°, 180°, 270°) to look in that direction for person detection.

**Servo control:** GPIO via `gpiozero` (Pi 5 backend: `lgpio`)

```bash
sudo apt install python3-gpiozero python3-lgpio
```

---

## Common Workflows

### Full navigation run (after map is saved)

```bash
# Terminal 1
python3 ~/ros2_ws/src/hamerschlag_haul/hamerschlag_haul/otos_odom_node.py

# Terminal 2
ros2 launch ~/ros2_ws/src/hamerschlag_haul/launch/master_launch.py
```

### Remap the environment

```bash
# Terminal 1
python3 ~/ros2_ws/src/hamerschlag_haul/hamerschlag_haul/otos_odom_node.py

# Terminal 2
ros2 launch ~/ros2_ws/src/hamerschlag_haul/launch/cartographer_launch.py

# When done, save
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/hhaul/map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Check everything is publishing

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /person_detections
```

### View the TF tree

```bash
ros2 run tf2_tools view_frames
```

---

## Networking

- **Pi SSH:** `ssh hhaul@172.26.19.241`
- **Raspberry Pi Connect (browser):** https://connect.raspberrypi.com
- **ROS_DOMAIN_ID:** 42 (set on both Pi and KR260)
- **Detection TCP port:** 5555 (KR260 -> Pi)
- **CMU network:** WPA2-Enterprise (CMU Secure), configured via `nmcli`

### Force IPv4 for apt (CMU network fix)

```bash
echo 'Acquire::ForceIPv4 "true";' | sudo tee /etc/apt/apt.conf.d/99force-ipv4
```

---

## Git

**Remote:** `git@github.com:Justin-Meme-Sometimes/hhaul.git`

```bash
# Set up SSH key on Pi
ssh-keygen -t ed25519 -C "youremail@gmail.com"
cat ~/.ssh/id_ed25519.pub  # Add to GitHub Settings -> SSH keys

# Set identity
git config --global user.name "YourGitHubUsername"
git config --global user.email "youremail@gmail.com"

# Verify remote uses SSH not HTTPS
git remote -v
git remote set-url origin git@github.com:Justin-Meme-Sometimes/hhaul.git
```

---

## Troubleshooting

### Map drifts or jumps

- OTOS must be mounted rigidly — handheld testing is useless for SLAM
- Check `published_frame = "odom"` in the Cartographer lua (not `"base_link"`)
- Verify no duplicate TF broadcasts: `ros2 run tf2_tools view_frames`
- LiDAR height matters — 15-20cm off the floor, never at floor level

### TF lookup failures in Cartographer

- Check `robot_state_publisher` has `ignore_timestamp: True` in its parameters
- Verify OTOS node is running and publishing to `/odom` before starting Cartographer
- OTOS QoS must be BEST_EFFORT, not RELIABLE

### Nav2 costmaps not appearing in RViz2

- Nav2 lifecycle manager takes ~10 seconds to activate after launch
- Check: `ros2 node list | grep nav2`
- Check: `ros2 topic list | grep costmap`

### Arduino not responding

- Check serial port: `ls /dev/ttyUSB*`
- Try both `/dev/ttyUSB0` and `/dev/ttyUSB1`
- Watchdog trip: if no heartbeat for 1s, motors stop and Arduino sends `<ERR,watchdog_stop\n>`
- Test standalone: `python3 test_serial.py`

### Person detections not arriving on Pi

- Start Pi listener before KR260 sender
- Check same ROS_DOMAIN_ID=42 on both devices
- Verify TCP: `netstat -tlnp | grep 5555` on Pi

### apt failing on CMU network

```bash
echo 'Acquire::ForceIPv4 "true";' | sudo tee /etc/apt/apt.conf.d/99force-ipv4
sudo apt update
```
