# SLAM Robot — ROS 2 Autonomous Inspection Robot

## Project Goal
Mobile robot that builds a map using LiDAR + SLAM Toolbox, navigates via Nav2, avoids obstacles,
drives physical motors via STM32, reads encoders, and explores unknown maps autonomously using
frontier-based exploration.

---

## Directory Layout

```
~/Documents/ROS/                            ← this repo (local mirror / config files)
├── CLAUDE.md
├── STM codebase/
│   ├── main.c                              ← full STM32 firmware (FreeRTOS, motors, encoders)
│   └── ahmedelsisi.ioc                     ← STM32CubeMX project config
├── nav2_params/
│   ├── nav2_params.yaml                    ← active Nav2 config (471 lines)
│   └── nav2_params_backup_before_tf_fix.yaml
├── explore_params/
│   └── frontier_params.yaml                ← active frontier config (54 lines)
└── ros2_ws/src/
    ├── cmd_vel_to_stm32/                   ← custom Python bridge package
    ├── my_robot_description/               ← URDF
    ├── ros2_laser_scan_matcher/            ← virtual odom node (C++)
    ├── frontier_exploration_ros2/          ← autonomous exploration (C++, v1.6.0)
    └── csm/                               ← ICP library dependency

ON THE PI (~/  paths):
  ~/slam_virtual_odom.yaml                  ← SLAM Toolbox params (NOT in this repo)
  ~/maps/                                   ← saved maps go here
  ~/ros2_ws/                                ← built main workspace
  ~/ws_lidar/src/sllidar_ros2/              ← RPLIDAR A1 driver (built)
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Main computer | Raspberry Pi 5 |
| OS | Ubuntu 24.04 LTS 64-bit |
| ROS | ROS 2 Jazzy |
| Microcontroller | STM32 Nucleo-F446RE |
| LiDAR | Slamtec RPLIDAR A1 (A1M8) |
| Motors | 4× DC motors |
| Motor drivers | 2× L298N |
| Encoders | 2× wheel encoders (front-left, front-right) |
| Pi↔STM32 | UART `/dev/ttyAMA0` at 115200 baud |
| LiDAR USB | `/dev/ttyUSB0` |

## Robot Geometry (confirmed in URDF)

| Measurement | Value | URDF derivation |
|-------------|-------|-----------------|
| Length | 38 cm | base_link box size |
| Width | 17 cm | base_link box size |
| Chassis height | 12 cm | base_link box size |
| Ground clearance | 5.1 cm | |
| Wheel diameter | 65 mm (r=32.5 mm) | |
| Wheel width | 25.3 mm | |
| Track width | 21.5 cm → y=±0.1075 m | wheel joint y |
| Wheelbase | 21.3 cm → x=±0.1065 m | wheel joint x |
| Wheel center height | 32.5 mm | |
| base_link z from ground | 111 mm | 0.051 + 0.12/2 |
| Wheel joint z (rel base_link) | −78.5 mm | 0.0325 − 0.111 |
| LiDAR height from ground | 238 mm | |
| LiDAR z (rel base_link) | 127 mm | 0.238 − 0.111 |
| LiDAR x offset | +10 mm | |

---

## ROS Frames

Required TF tree:
```
map → odom → base_footprint → base_link → wheels
                            ↘ laser
```

| Publisher | Transform(s) |
|-----------|-------------|
| `robot_state_publisher` (URDF) | `base_footprint → base_link` (fixed, z=+0.111 m) |
| URDF | `base_link → laser` (fixed, xyz=0.01 0 0.127) |
| URDF | `base_link → {front/rear}_{left/right}_wheel` (continuous) |
| `ros2_laser_scan_matcher` | `odom → base_footprint` (virtual, from scan matching) |
| `slam_toolbox` | `map → odom` |

**CRITICAL:** Nav2 must use `robot_base_frame: base_footprint` everywhere — confirmed set throughout nav2_params.yaml. Never change to `base_link`.

**NOTE:** AMCL is present in nav2_params.yaml but is NOT used. SLAM Toolbox provides `map → odom` directly. Do not launch AMCL.

---

## Environment (source in every terminal)

```bash
source /opt/ros/jazzy/setup.bash
source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

LiDAR terminals also need:
```bash
source /home/slamrobot/ws_lidar/install/setup.bash
```

---

## Terminal Launch Order

### Terminal 0 — Kill old processes (run first every session)
```bash
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f sllidar
pkill -f laser_scan_matcher
pkill -f slam_toolbox
pkill -f nav2
pkill -f controller_server
pkill -f planner_server
pkill -f bt_navigator
pkill -f velocity_smoother
pkill -f collision_monitor
pkill -f rviz2
pkill -f cmd_vel_bridge
pkill -f frontier_explorer
pkill -f frontier_exploration_ctl
pkill -f minicom
pkill -f picocom

sudo lsof /dev/ttyAMA0
sudo lsof /dev/ttyUSB0
```

### Terminal 1 — robot_state_publisher
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat /home/slamrobot/ros2_ws/src/my_robot_description/urdf/robot.urdf)"
```

### Terminal 2 — joint_state_publisher
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run joint_state_publisher joint_state_publisher
```

### Terminal 3 — LiDAR
```bash
source /opt/ros/jazzy/setup.bash
source /home/slamrobot/ws_lidar/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch sllidar_ros2 sllidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=115200 \
  frame_id:=laser
```
Launch defaults: `channel_type=serial`, `inverted=false`, `angle_compensate=true`, `scan_mode=Sensitivity`.

### Terminal 4 — Virtual Odom (laser_scan_matcher)
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run ros2_laser_scan_matcher laser_scan_matcher --ros-args \
  -p base_frame:=base_footprint \
  -p odom_frame:=odom \
  -p publish_tf:=true \
  -p publish_odom:=/odom
```

### Terminal 5 — SLAM Toolbox
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/slamrobot/slam_virtual_odom.yaml
```
Config file lives on the Pi at `~/slam_virtual_odom.yaml` — not in this repo.

### Terminal 6 — Motor + Encoder Bridge
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run cmd_vel_to_stm32 cmd_vel_bridge
```
No remapping needed — bridge subscribes directly to `/cmd_vel_out` (collision monitor output).
To test with L/R physically inverted, add: `--ros-args -p invert_rotation:=true`

### Terminal 7 — Nav2
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/slamrobot/nav2_params/nav2_params.yaml
```

### Terminal 8 — RViz2
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rviz2
```
Settings: Fixed Frame = `map`
Add displays: TF, RobotModel, LaserScan (`/scan`), Map (`/map`), Global Costmap, Local Costmap, Path, MarkerArray (`/explore/frontiers`), Pose (`/explore/selected_frontier` — type is `geometry_msgs/msg/PoseStamped`).

### Terminal 9 — Frontier Explorer
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run frontier_exploration_ros2 frontier_explorer \
  --ros-args \
  --params-file /home/slamrobot/explore_params/frontier_params.yaml \
  -p use_sim_time:=false
```

### Terminal 10 — Start/Stop Autonomous Exploration
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Start
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start

# Stop
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop
```
`autostart: false` in frontier_params.yaml — manual start required every time.

### Emergency Stop
```bash
ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
# or:
pkill -f cmd_vel_bridge
```

---

## Package Details

### cmd_vel_to_stm32 (Python, ~/ros2_ws/src/cmd_vel_to_stm32/)

Subscribes to `/cmd_vel_out` (collision monitor final output — full safety pipeline), sends ASCII
commands to STM32 over `/dev/ttyAMA0` at 115200, reads encoder lines, publishes `/encoder_data`.

**Nav2 velocity pipeline (confirmed):**
```
controller_server → /cmd_vel
velocity_smoother → /cmd_vel_smoothed
collision_monitor → /cmd_vel_out   ← bridge reads this
STM32             ← bridge writes ASCII char
```

**cmd_vel_bridge.py — current logic:**
```
linear_x > 0.05 AND angular_z small  → 'F'         # straight forward
linear_x < -0.05 AND angular_z small → 'B'         # straight backward
angular_z only                        → 'L' or 'R'  # pure spin
BOTH linear AND angular active        → time-sliced 'F'/'L'/'R' per blend ratio
else                                  → 'S'         # stop
```
STM32 'F' = left_forward + right_forward = physical forward ✓  
STM32 'B' = left_reverse + right_reverse = physical backward ✓

**Combined velocity — turn-to-align with hysteresis:** When Nav2 MPPI sends both `linear_x`
and `angular_z`, the bridge uses a two-threshold state machine. If `|angular_z|/WZ_MAX` exceeds
`ALIGN_TURN_START` (0.35 ≈ 0.67 rad/s), the bridge commits to an in-place turn. It resumes
driving only once `|angular_z|/WZ_MAX` drops below `ALIGN_DRIVE_START` (0.15 ≈ 0.29 rad/s).
The hysteresis gap prevents oscillation between turn and drive. Entering alignment turn clears
`last_motion` so no spurious brake pulse fires after a pure spin.

**Counter-brake on stop:** When transitioning from `F`/`B` to `S`, the bridge sends a 60 ms
counter-pulse in the opposite direction before issuing the final `S`. Implemented via a ROS timer;
any non-stop command arriving during the pulse cancels the brake immediately. `BRAKE_PULSE_S = 0.060`
in `cmd_vel_bridge.py` — reduce if the robot creeps backward, increase if it still coasts.

**invert_rotation parameter:** If physical L/R is backwards, run with
`--ros-args -p invert_rotation:=true` to swap without rebuilding.

**Watchdog:** Sends `'S'` to STM32 if no `/cmd_vel_out` arrives for >1 s.

Backup of original (pre-swap) mapping: `cmd_vel_bridge_backup_before_fb_swap.py`
Serial read timer: 50 Hz (0.02 s). On shutdown: sends `'S'` before closing port.

**Encoder format from STM32:**
```
ENC,seq,time_ms,left_delta,right_delta,left_total_ticks,right_total_ticks
```
Published to `/encoder_data` as `std_msgs/msg/Int64MultiArray`:
- `data[0]` = seq
- `data[1]` = time_ms
- `data[2]` = left_delta
- `data[3]` = right_delta
- `data[4]` = left_total_ticks
- `data[5]` = right_total_ticks

Parser expects exactly 7 comma-separated fields. Non-ENC lines are logged as warnings and skipped.

**SERIAL RULE: Only one process may hold `/dev/ttyAMA0` at a time. Never run minicom/picocom alongside cmd_vel_bridge.**

---

### my_robot_description (CMake, ~/ros2_ws/src/my_robot_description/)

URDF: `urdf/robot.urdf` — complete static TF tree for RSP.

**Joint summary:**
| Joint | Type | Parent | Child | xyz | rpy |
|-------|------|--------|-------|-----|-----|
| base_footprint_to_base_link | fixed | base_footprint | base_link | 0 0 0.111 | 0 0 0 |
| laser_joint | fixed | base_link | laser | 0.01 0 0.127 | 0 0 0 |
| front_left_wheel_joint | continuous | base_link | front_left_wheel | 0.1065 0.1075 -0.0785 | 1.5708 0 0 |
| front_right_wheel_joint | continuous | base_link | front_right_wheel | 0.1065 -0.1075 -0.0785 | 1.5708 0 0 |
| rear_left_wheel_joint | continuous | base_link | rear_left_wheel | -0.1065 0.1075 -0.0785 | 1.5708 0 0 |
| rear_right_wheel_joint | continuous | base_link | rear_right_wheel | -0.1065 -0.1075 -0.0785 | 1.5708 0 0 |

---

### ros2_laser_scan_matcher (C++, ~/ros2_ws/src/ros2_laser_scan_matcher/)

ICP-based virtual odometry. Depends on `csm` (in same workspace, branch `ros2_csm_eigen`).
Parameters used: `base_frame=base_footprint`, `odom_frame=odom`, `publish_tf=true`, `publish_odom=/odom`.

---

### frontier_exploration_ros2 (C++, v1.6.0, ~/ros2_ws/src/frontier_exploration_ros2/)

Executables: `frontier_explorer`, `frontier_exploration_ctl`, `frontier_debug_observer`
Service: `/control_exploration` (type `ControlExploration.srv` — action START=1 / STOP=2)
Strategy: MRTSP with greedy solver. Visualization on `/explore/frontiers` and `/explore/selected_frontier`.

---

### sllidar_ros2 (C++, ~/ws_lidar/src/sllidar_ros2/)

RPLIDAR A1 driver. Publishes `/scan` with `frame_id=laser`.
Launch: `sllidar_a1_launch.py` — supports 26 RPLIDAR model variants.

---

## Key Config Values (confirmed from files)

### nav2_params.yaml — important settings
| Setting | Value | Location |
|---------|-------|----------|
| `robot_base_frame` | `base_footprint` | amcl, bt_navigator, local_costmap, global_costmap, collision_monitor, behavior_server |
| `transform_tolerance` | `0.5` | amcl, MPPI, collision_monitor, behavior_server, docking_server |
| Controller | MPPI (`nav2_mppi_controller::MPPIController`) | controller_server |
| `motion_model` | `DiffDrive` | MPPI |
| `vx_max` | `0.5 m/s` | MPPI |
| `wz_max` | `1.9 rad/s` | MPPI |
| `vx_min` | `-0.35 m/s` | MPPI |
| `robot_radius` (local costmap) | `0.22 m` | local_costmap |
| `robot_radius` (global costmap) | `0.25 m` | global_costmap |
| `inflation_radius` (local costmap) | `0.40 m` | local_costmap |
| `inflation_radius` (global costmap) | `0.20 m` | global_costmap |
| Costmap resolution | `0.05 m/cell` | local + global costmap |
| Local costmap layer | `ObstacleLayer` (was VoxelLayer — switched to reduce Pi 5 CPU) | local_costmap |
| Local costmap size | 3×3 m, rolling window | local_costmap |
| `obstacle_max_range` | `2.5 m` | both costmaps |
| `raytrace_max_range` | `3.0 m` | both costmaps |
| Planner | NavFn, `use_astar: false`, `allow_unknown: true` | planner_server |
| `max_velocity` | `[0.5, 0.0, 2.0]` | velocity_smoother |
| `max_accel` | `[2.5, 0.0, 3.2]` | velocity_smoother |
| Collision monitor | `FootprintApproach`, `time_before_collision: 4.8 s`, output → `/cmd_vel_out` | collision_monitor |

Backup: `nav2_params_backup_before_tf_fix.yaml` — state before transform_tolerance was raised to 0.5.

### frontier_params.yaml — confirmed values
| Setting | Value |
|---------|-------|
| `autostart` | `false` (manual start required) |
| `control_service_enabled` | `true` |
| `strategy` | `mrtsp` |
| `mrtsp_solver` | `greedy` |
| `sensor_effective_range_m` | `3.0` |
| `frontier_candidate_min_goal_distance_m` | `0.25` |
| `frontier_selection_min_distance` | `0.35` |
| `frontier_visit_tolerance` | `0.30` |
| `min_frontier_size_cells` | `8` |
| `occ_threshold` | `60` |
| `escape_enabled` | `true` |
| `return_to_start_on_complete` | `false` |
| `goal_skip_on_blocked_goal` | `true` |
| `goal_preemption_enabled` | `true` |
| `goal_preemption_min_interval_s` | `2.0` |
| `goal_preemption_complete_if_within_m` | `0.5` |
| `goal_preemption_lidar_range_m` | `3.0` |
| `map_qos_durability` | `transient_local` |

---

## Validation Commands

```bash
# LiDAR
ros2 topic echo /scan --once | grep frame_id
ros2 topic hz /scan

# TF — debug in this order every time
ros2 run tf2_ros tf2_echo base_footprint laser
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo odom laser
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint

# Nav2 lifecycle (all must show 'active')
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother

# Frontier exploration
ros2 node list | grep frontier
ros2 service list | grep control        # expect: /control_exploration
ros2 topic list | grep explore

# Encoders
ros2 topic echo /encoder_data --once
ros2 topic hz /encoder_data

# cmd_vel flow — monitor the full pipeline
ros2 topic echo /cmd_vel_smoothed       # velocity_smoother output
ros2 topic echo /cmd_vel_out            # collision_monitor output → bridge reads this
```

---

## Manual Motor Tests (⚠ lift wheels off ground first!)

> These publish directly to `/cmd_vel_out` (what the bridge reads), bypassing Nav2 entirely.
> Use only with cmd_vel_bridge running and Nav2 stopped.

```bash
# Forward
timeout 2 ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Backward
timeout 2 ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: -0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Turn — angular_z positive (currently mapped to 'L')
timeout 2 ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}" -r 10

# Turn — angular_z negative (currently mapped to 'R')
timeout 2 ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.30}}" -r 10

# Stop
ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

If both angular_z directions produce the same physical turn: swap `'L'` and `'R'` in
`~/ros2_ws/src/cmd_vel_to_stm32/cmd_vel_to_stm32/cmd_vel_bridge.py` lines 48 and 50,
then rebuild: `cd ~/ros2_ws && colcon build --packages-select cmd_vel_to_stm32`.

---

## Reset Map (start fresh without restarting everything)

SLAM Toolbox has no live reset service in online async mode. Only Terminal 5 needs to restart — the rest of the TF chain stays intact.

**Step 1 — kill SLAM Toolbox:**
```bash
pkill -f slam_toolbox
```

**Step 2 — restart SLAM Toolbox (Terminal 5):**
```bash
source /opt/ros/jazzy/setup.bash && source /home/slamrobot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/slamrobot/slam_virtual_odom.yaml
```

**Step 3 — clear Nav2 costmaps** (removes stale obstacle data from the old map):
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

**Step 4 — if frontier explorer was running, restart it (Terminal 9):**
```bash
pkill -f frontier_explorer
```
Then relaunch Terminal 9 so it doesn't navigate toward frontiers from the old map.

RViz will automatically show the new empty map once SLAM Toolbox republishes `map → odom`.
Do NOT restart `laser_scan_matcher`, `robot_state_publisher`, or `joint_state_publisher`.

---

## Save Map
```bash
mkdir -p /home/slamrobot/maps
ros2 run nav2_map_server map_saver_cli -f /home/slamrobot/maps/autonomous_explored_map
# Produces: autonomous_explored_map.yaml + autonomous_explored_map.pgm
```

---

## Remote Supervision

Full setup instructions are in [REMOTE_SUPERVISION.md](REMOTE_SUPERVISION.md).

**Summary:** Both devices join the same local hotspot. Pi runs x11vnc + noVNC as systemd
services (auto-start on boot). Supervisor opens a browser and visits:
```
http://<PI_IP>:6080/vnc.html
```
No internet required. No software needed on the supervisor device.

Find Pi's IP: `hostname -I`

---

## Common Issues and Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| RViz: `Frame [map] does not exist` | SLAM Toolbox not running or no map→odom yet | Start SLAM Toolbox after LiDAR + odom confirmed working |
| SLAM: `Message Filter dropping message: frame 'laser'` | TF chain broken | Debug full chain in order above |
| Nav2: `Timed out waiting for transform from base_link` | Wrong base frame | Confirm `robot_base_frame: base_footprint` everywhere in nav2_params.yaml |
| Nav2: `Lookup would require extrapolation into the future` | TF timestamp drift | Already fixed: `transform_tolerance: 0.5` set throughout |
| Nav2: `Failed to create plan with tolerance 0.500000` | Goal in obstacle / unknown / inflated zone | Clear costmaps; send goal to clear open area |
| Robot turns only one direction | L/R mapping wrong | Echo `/cmd_vel_out`; run manual turn tests; use `-p invert_rotation:=true` on bridge |
| `Service 'control_exploration' not available` | frontier_explorer not running or crashed | `ros2 node list \| grep frontier`; restart Terminal 9 |

### Clear costmaps
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

### Fix Nav2 base_frame if accidentally changed
```bash
sed -i 's/robot_base_frame: base_link/robot_base_frame: base_footprint/g' \
  /home/slamrobot/nav2_params/nav2_params.yaml
```

### Rebuild after editing Python bridge
```bash
cd ~/ros2_ws && colcon build --packages-select cmd_vel_to_stm32
source ~/ros2_ws/install/setup.bash
```

### Rebuild everything
```bash
cd ~/ros2_ws && colcon build
source ~/ros2_ws/install/setup.bash
```

---

## STM32 Firmware (STM codebase/main.c)

**MCU:** STM32F446RETx (Nucleo-F446RE), LQFP64  
**RTOS:** FreeRTOS CMSIS v2  
**System clock:** 84 MHz (HSI 16 MHz → PLL: PLLM=16, PLLN=336, PLLP=DIV4)  
**Toolchain:** STM32CubeIDE / GCC, Firmware: STM32Cube FW_F4 V1.28.3

### FreeRTOS Tasks

| Task | Priority | Stack | Role |
|------|----------|-------|------|
| `defaultTask` | Normal | 128×4 B | Polls `current_cmd` every 10 ms, calls `apply_cmd()` on change |
| `controlTask` | AboveNormal | 128×4 B | Blocks on `cmdQueueHandle`, sets `current_cmd`, logs "CMD: X" to USART2 |
| `encoderTask` | Low | 256×4 B | Reads both encoders every 50 ms, sends ENC frame via UART4, DBG to USART2 |

### UART Peripherals

| Peripheral | Pins | Baud | Purpose |
|------------|------|------|---------|
| **UART4** | PA0 (TX), PA1 (RX) | 115200 | **Pi ↔ STM32** — receives commands, transmits ENC frames |
| **USART2** | PA2 (TX), PA3 (RX) | 115200 | ST-Link virtual COM (debug) — DBG frames, CMD echo, RX echo |

**UART4 ISR behavior:** 1-byte interrupt-driven RX. Lowercase auto-converted to uppercase. Valid chars `F/B/L/R/S` pushed to `cmdQueueHandle` (depth 8). Raw debug echo `RX:0xXX ('c')` sent to USART2. Error callback clears PE/FE/NE/ORE flags and re-arms `Receive_IT`.

### PWM — Motor Speed

| Timer | Channel | Pin | Label | Config |
|-------|---------|-----|-------|--------|
| TIM4 | CH1 | PB6 | D10_EN | Prescaler=83, Period=49 → **20 kHz**, shared EN for all motors |

Speed constants:
- `SPEED_DRIVE = 49` → 100% duty → full speed (straight)
- `SPEED_TURN  = 32` → ~65% duty → reduced speed (rotation only)

PWM frequency: 84 MHz ÷ (83+1) ÷ (49+1) = **20 kHz**

### Encoders

| Timer | Pins | Assignment |
|-------|------|------------|
| TIM1 | PA8 (CH1), PA9 (CH2) | **Left encoder** (TIM1 = left per code comment) |
| TIM8 | PC6 (CH1), PC7 (CH2) | **Right encoder** |

Both: TI12 quadrature mode, both edges FALLING, period=65535 (16-bit counter).  
Delta computation uses 16-bit signed cast for wraparound: `(int16_t)(now - previous)`.  
**Both deltas are negated** before accumulation: `left_delta = -encoder_delta_16bit(...)` — compensates physical mounting direction.  
Encoder assignment comment in code: *"If your physical robot shows the opposite, swap htim1 and htim8."*

Encoder constants:
```c
TICKS_PER_REV      = 3960.0f
WHEEL_DIAMETER     = 6.7 cm
WHEEL_CIRCUMFERENCE = π × 6.7 ≈ 21.05 cm
CM_PER_TICK        = 21.05 / 3960 ≈ 0.005315 cm/tick
```

### Motor GPIO Pinout

| Motor | Side | IN1 pin | IN2 pin | Notes |
|-------|------|---------|---------|-------|
| M1 front-left | Left | PA10 (D2) | PB5 (D4) | |
| M2 front-right | Right | PB4 (D5) | PB10 (D6) | |
| M3 rear-left | Left | PB14 (IN1_2) | PB15 (IN2_2) | Wired inverted vs M1 — same-side motors face opposite directions |
| M4 rear-right | Right | PA6 (D12) | PA5 (D13) | Wired inverted vs M2 — same-side motors face opposite directions |

M1 and M3 (left side) have **opposite IN1/IN2 polarity** for the same direction. Same for M2/M4. This is intentional — front and rear motors on each side are physically mounted facing opposite directions.

### apply_cmd() Logic (confirmed from code)

| Char | Left side | Right side | Speed |
|------|-----------|------------|-------|
| `'F'` | forward | forward | SPEED_DRIVE (49) | physical forward ✓ confirmed |
| `'B'` | reverse | reverse | SPEED_DRIVE (49) | physical backward ✓ confirmed |
| `'R'` | forward | reverse | SPEED_TURN (32) — turns right in place |
| `'L'` | reverse | forward | SPEED_TURN (32) — turns left in place |
| `'S'` | brake | brake | 0 |

Stop = both INx LOW (L298N brake, not coast).

### ENC Frame Format (sent by encoderTask via UART4, 20 Hz)

```
ENC,<seq>,<time_ms>,<left_delta>,<right_delta>,<left_total>,<right_total>\r\n
```
- `seq`: 32-bit counter starting at 0
- `time_ms`: `HAL_GetTick()` milliseconds since boot
- deltas and totals: signed 32-bit integers
- UART4 transmit timeout: 20 ms per frame

### DBG Frame Format (sent to USART2 only — not seen by Pi)

```
DBG,Ld:<left_delta> Rd:<right_delta> Ltot:<left_total> Rtot:<right_total> Lcm:<left_cm> Rcm:<right_cm>\r\n
```

### ⚠ Important Firmware Notes

1. **Combined velocity handled by time-slicing.** The STM32 still only accepts one command at a time (F/B/L/R/S). When Nav2 MPPI sends combined linear+angular, the bridge time-slices between forward and turn commands over a 4-tick window proportional to the normalised magnitudes. This approximates arcing; true differential drive requires a hardware PWM split (future work).

2. **Encoder assignment may need swap.** TIM1=left, TIM8=right is assumed in code but has a comment warning to swap if physical behavior is wrong.

3. **Both encoder deltas are negated.** Negative delta = forward movement for both wheels (as mounted).

4. **UART4 is the Pi link.** USART2 is debug-only (ST-Link). Never connect Pi to USART2.

5. **STM32 accepts lowercase commands** — UART ISR auto-uppercases them.

6. **Startup message:** On boot, STM32 sends `DBG,UART4 command link ready\r\n` on both USART2 and UART4.

---

## Known Open Items

- [ ] **L/R direction needs physical verification.** Code logic is correct (angular_z > 0 → 'L' → right_fwd+left_rev → CCW = left in ROS). If robot turns the wrong way, start bridge with `--ros-args -p invert_rotation:=true` to test; if that fixes it, make it permanent.
- [ ] **True differential drive requires hardware.** Time-slice blend approximates arcing but the STM32 has only one shared PWM EN pin (TIM4 CH1 / PB6). Real arc motion needs a second PWM channel on the second L298N EN plus firmware + protocol changes.
- [ ] **Encoder left/right assignment not physically verified.** TIM1=left, TIM8=right per code comment — confirm by spinning one wheel and checking `/encoder_data` fields.
- [ ] `slam_virtual_odom.yaml` lives on the Pi only — not tracked in this repo. Copy it here.
- [ ] Encoder odometry not yet fused into `/odom`. `/odom` comes entirely from laser_scan_matcher; `/encoder_data` is published but unused in the TF chain.
- [ ] `docking_server` in nav2_params.yaml uses `base_frame: base_link` — harmless, docking not used.

---

## Project Rules (for AI assistant)

1. Always source both `/opt/ros/jazzy/setup.bash` and `~/ros2_ws/install/setup.bash` in every terminal.
2. LiDAR terminals also source `~/ws_lidar/install/setup.bash`.
3. Never assume a node is running — verify with `ros2 node list`, `topic list`, `service list`, or `tf2_echo`.
4. Debug TF in order: `base_footprint→laser` → `odom→base_footprint` → `odom→laser` → `map→odom` → `map→base_footprint`.
5. Virtual odom comes from `ros2_laser_scan_matcher`. Encoders are published but NOT fused into odom yet.
6. Nav2 `robot_base_frame` = `base_footprint` everywhere. Never `base_link`.
7. Only one process may hold `/dev/ttyAMA0` at a time.
8. Lift wheels before any manual motor test command.
9. B = forward, F = backward (physically swapped from STM32 convention).
10. L/R direction needs physical verification; use `-p invert_rotation:=true` on the bridge to test without rebuilding.
11. Bridge reads `/cmd_vel_out` (collision_monitor output). Do not re-add the old `/cmd_vel_smoothed` remapping.
12. STM32 watchdog stops motors after 500 ms of no UART4 command; bridge watchdog sends 'S' after 1 s of no `/cmd_vel_out`.
11. AMCL is in nav2_params.yaml but is NOT used — SLAM Toolbox provides map→odom.
12. `frontier_exploration_ctl` must be run via `ros2 run`, not as a bare shell command.
