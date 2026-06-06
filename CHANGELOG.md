# Changelog — SLAM Robot — ahmed-adly Branch
## Approach 1 Algorithms + Approach 2 (Saif) Parameter Values

Code and algorithms: `fix/critical-navigation-and-safety` (Approach 1)
Nav2 parameter values: `Saif` (Approach 2)

All notable changes, fixes, and known issues are tracked here.
Format: `[Status] | Issue | Severity | Fix Location | Date`

---

## Pending Fixes

None — all critical and high-priority fixes from the initial audit are implemented.

---

## Unverified / Open Items

| # | Item | Notes |
|---|------|-------|
| A | L/R direction not physically confirmed | Use `--ros-args -p invert_rotation:=true` on bridge to test; make permanent if needed |
| B | `slam_virtual_odom.yaml` not tracked in this repo — lives on Pi only | Copy from Pi and commit |
| C | Encoder left/right assignment (TIM1=left, TIM8=right) not confirmed | Spin one wheel, check `/encoder_data` fields |
| D | Encoder odometry not fused into `/odom` — encoders published but unused in TF chain | Future work: implement differential drive odometry node |
| E | True differential drive requires hardware change | Second PWM channel needed for second L298N EN; current single shared PWM (TIM4 CH1) prevents real arcing |

---

## Completed Changes

| # | Status | Change | Date |
|---|--------|--------|------|
| 1 | ✅ Done | Forward/backward physically swapped in `cmd_vel_bridge.py` — `B`=forward, `F`=backward | 2026-05-04 |
| 14 | ✅ Done | **[Hardware] F/B corrected back to `F`=forward, `B`=backward** — physical testing confirmed STM32 `'F'` (left_forward+right_forward) = forward, `'B'` (left_reverse+right_reverse) = backward; previous swap no longer matched firmware; both blend and linear-only paths corrected | 2026-05-21 |
| 2 | ✅ Done | `transform_tolerance` increased to 0.5 across all Nav2 nodes — fixed TF extrapolation errors | 2026-05-04 |
| 3 | ✅ Done | `robot_base_frame` set to `base_footprint` everywhere in `nav2_params.yaml` | 2026-05-04 |
| 4 | ✅ Done | Full URDF created with correct geometry — `base_footprint→base_link→laser`, all 4 wheels | 2026-05-04 |
| 5 | ✅ Done | `ros2_laser_scan_matcher` integrated as virtual odometry source — publishes `odom→base_footprint` | 2026-05-04 |
| 6 | ✅ Done | `frontier_exploration_ros2` integrated and configured — MRTSP/greedy strategy, manual start | 2026-05-04 |
| 7 | ✅ Done | **[Safety] STM32 motor watchdog added** — `defaultTask` stops motors (`current_cmd='S'`) if no valid UART4 command received for >500 ms; `last_cmd_time_ms` seeded on boot and updated in ISR | 2026-05-21 |
| 8 | ✅ Done | **[Safety] Bridge watchdog added** — `cmd_vel_bridge.py` sends `'S'` to STM32 if no `/cmd_vel_out` message received for >1 s | 2026-05-21 |
| 9 | ✅ Done | **[Safety] Collision monitor output now reaches STM32** — bridge changed to subscribe to `/cmd_vel_out`; `collision_monitor.cmd_vel_out_topic` changed from `"cmd_vel"` to `"cmd_vel_out"` in `nav2_params.yaml`; eliminates dual-publisher conflict on `/cmd_vel` | 2026-05-21 |
| 10 | ✅ Done | **[Navigation] Combined velocity blend implemented** — bridge time-slices between forward and turn commands over a 4-tick (200 ms) window when both `linear_x` and `angular_z` are active; ratio proportional to normalised magnitudes (VX_MAX=0.5, WZ_MAX=1.9) | 2026-05-21 |
| 11 | ✅ Done | **[Navigation] `invert_rotation` parameter added to bridge** — swap L/R physically without recompiling: `--ros-args -p invert_rotation:=true` | 2026-05-21 |
| 12 | ✅ Done | **[Navigation] Inflation radius reduced 0.70 m → 0.35 m** in both local and global costmaps — prevents planner blocking standard doorways on a 17 cm wide robot | 2026-05-21 |
| 13 | ✅ Done | **[Performance] Local costmap VoxelLayer → ObstacleLayer** — removes unnecessary 3D voxel processing for a 2D LiDAR robot, reduces Pi 5 CPU and memory usage | 2026-05-21 |
