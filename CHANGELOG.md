# Changelog — SLAM Autonomous Inspection Robot

All notable changes, fixes, and known issues are tracked here.
Format: `[Status] | Issue | Severity | Fix Location | Date`

---

## Pending Fixes

| # | Status | Issue | Severity | Fix Location |
|---|--------|-------|----------|-------------|
| 1 | 🔴 Pending | No motor watchdog — robot drives forever if Pi crashes or UART drops | **Critical / Safety** | STM32 `main.c` + `cmd_vel_bridge.py` |
| 2 | 🔴 Pending | Collision monitor output not reaching STM32 — bridge reads `/cmd_vel_smoothed` (pre-monitor), safety stops discarded | **Critical / Safety** | Bridge launch command (remove `/cmd_vel` remapping) |
| 3 | 🔴 Pending | L/R turn direction not physically verified — both directions may produce the same rotation | **Critical / Navigation** | Physical test + `cmd_vel_bridge.py` lines 48/50 |
| 4 | 🔴 Pending | Combined velocity silently dropped — Nav2 MPPI sends linear+angular together, bridge discards angular when `linear_x > 0.05`, robot cannot arc | **High / Navigation** | `cmd_vel_bridge.py` short-term blend fix; hardware PWM split long-term |
| 5 | 🔴 Pending | Inflation radius 0.70 m blocks doorways — robot width is 17 cm, planner will refuse paths through standard doorways | **High / Navigation** | `nav2_params/nav2_params.yaml` → reduce to 0.35 m |
| 6 | 🔴 Pending | VoxelLayer in local costmap wastes CPU on Pi 5 for a 2D LiDAR robot | **Medium / Performance** | `nav2_params/nav2_params.yaml` → switch to ObstacleLayer |

---

## Unverified / Open Items

| # | Item | Notes |
|---|------|-------|
| A | Encoder left/right assignment (TIM1=left, TIM8=right) not physically confirmed | Spin one wheel, check which `/encoder_data` field changes |
| B | `slam_virtual_odom.yaml` not tracked in this repo — lives on Pi only | Copy from Pi and commit |
| C | Encoder odometry not fused into `/odom` — encoders published to `/encoder_data` but unused in TF chain | Future work: implement differential drive odometry node |
| D | Single shared PWM (TIM4 CH1) prevents true differential speed control — hardware limitation | Future work: wire separate EN per L298N, add second PWM channel |

---

## Completed Changes

| # | Status | Change | Date |
|---|--------|--------|------|
| 1 | ✅ Done | Forward/backward physically swapped in `cmd_vel_bridge.py` — `B`=forward, `F`=backward | 2026-05-04 |
| 2 | ✅ Done | `transform_tolerance` increased to 0.5 across all Nav2 nodes — fixed TF extrapolation errors | 2026-05-04 |
| 3 | ✅ Done | `robot_base_frame` set to `base_footprint` everywhere in `nav2_params.yaml` | 2026-05-04 |
| 4 | ✅ Done | Full URDF created with correct geometry — `base_footprint→base_link→laser`, all 4 wheels | 2026-05-04 |
| 5 | ✅ Done | `ros2_laser_scan_matcher` integrated as virtual odometry source — publishes `odom→base_footprint` | 2026-05-04 |
| 6 | ✅ Done | `frontier_exploration_ros2` integrated and configured — MRTSP/greedy strategy, manual start | 2026-05-04 |
