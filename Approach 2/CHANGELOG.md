# Changelog — Approach 2 (Saif Branch)
## Differential PWM Arc Motion Attempt

All changes relative to Approach 1 (`fix/critical-navigation-and-safety` branch).
Format: `[Status] | Change | Location | Date`

---

## Critical Known Gaps (UNRESOLVED)

| # | Issue | Detail |
|---|-------|--------|
| G1 | **STM32 has no VEL packet parser** | Pi sends `VEL,left_pwm,right_pwm\r\n` but STM32 firmware still only accepts single-byte commands `F/B/L/R/S` — the new packet format is silently ignored |
| G2 | **Single shared PWM channel** | TIM4 CH1 / PB6 is the only EN pin shared across both L298N drivers — true independent left/right speed control is impossible without a hardware redesign (second PWM channel on second L298N EN) |

These two gaps mean Approach 2 **cannot produce differential arc motion** in its current state. G2 is a hardware constraint; G1 is the firmware work needed before the Pi-side bridge can be tested.

---

## Open Items (inherited from Approach 1, still unresolved)

| # | Item | Notes |
|---|------|-------|
| A | L/R direction not physically confirmed | Use `--ros-args -p invert_rotation:=true` on bridge to test |
| B | `slam_virtual_odom.yaml` not tracked in repo — lives on Pi only | Copy from Pi and commit |
| C | Encoder left/right assignment (TIM1=left, TIM8=right) not confirmed | Spin one wheel, check `/encoder_data` fields |
| D | Encoder odometry not fused into `/odom` | Future work: implement differential drive odometry node |

---

## Changes in This Approach

### Pi-Side Bridge — `ros2_ws/src/cmd_vel_to_stm32/cmd_vel_to_stm32/cmd_vel_bridge.py`

| # | Status | Change | Detail |
|---|--------|--------|--------|
| 1 | ✅ Implemented | **Discrete command protocol replaced with differential PWM** | Instead of sending single-byte `F/B/L/R/S`, the bridge now sends `VEL,left_pwm,right_pwm\r\n` packets. This enables true arc motion if the STM32 side implements the parser. |
| 2 | ✅ Implemented | **`twist_to_pwm()` — differential mixing** | `forward_pwm = f(linear_x)`, `turn_pwm = f(angular_z)`, then `left = forward − turn`, `right = forward + turn`. Replaces the old hysteresis state machine and time-slice blending. |
| 3 | ✅ Implemented | **`enforce_motor_minimums()` — deadband enforcement** | Lifts nonzero but sub-stall wheel commands to minimum PWM. Uses `MIN_MOVING_PWM=17` when both wheels move same direction, `MIN_TURNING_PWM=12` otherwise. Prevents wheel stall on arcs where the mix creates small values (e.g. L=29, R=5). |
| 4 | ✅ Implemented | **10 Hz rate limiter on normal velocity packets** | `CMD_SEND_INTERVAL_S = 0.10`. Stop commands (`VEL,0,0`) are always sent immediately without waiting for the interval. Prevents serial flooding while preserving responsive stops. |
| 5 | ✅ Implemented | **Watchdog sends `VEL,0,0` (not `'S'`)** | If no `/cmd_vel_out` arrives for >1 s, the watchdog sends the new packet format instead of the old single-byte stop character. |
| 6 | ✅ Implemented | **Non-ENC serial line suppression** | Non-ENC lines from STM32 are batched and only one warning is logged per 5 s window, preventing log spam during boot or firmware noise. |
| 7 | ✅ Implemented | **`invert_rotation` parameter retained** | `--ros-args -p invert_rotation:=true` still available to swap L/R without rebuilding. |
| 8 | ✅ Implemented | **Turn-to-align hysteresis removed** | No longer needed — differential mixing handles arcs natively (once G1 and G2 are resolved). |
| 9 | ✅ Implemented | **Counter-brake pulse removed** | Removed the 60 ms counter-pulse on stop transitions — differential drive can achieve cleaner electrical braking by driving both wheels briefly at opposite PWM. |

**Key constants:**

| Constant | Approach 2 Value | Approach 1 Equivalent |
|----------|------------------|-----------------------|
| `MAX_LINEAR_MPS` | 0.35 m/s | `VX_MAX = 0.5` (normalisation only) |
| `MAX_ANGULAR_RPS` | 0.80 rad/s | `WZ_MAX = 1.9` (normalisation only) |
| `MAX_PWM` | 35 | N/A (STM32-side constant in Approach 1) |
| `MIN_MOVING_PWM` | 17 | N/A |
| `MIN_TURNING_PWM` | 12 | N/A |
| `MAX_TURNING_PWM` | 24 | N/A |
| `CMD_SEND_INTERVAL_S` | 0.10 s (10 Hz) | No equivalent — every callback sent |
| `DEADZONE_LINEAR` | 0.02 m/s | 0.05 m/s (`DEADZONE_LINEAR`) |
| `DEADZONE_ANGULAR` | 0.05 rad/s | 0.05 rad/s (`DEADZONE_ANGULAR`) |

---

### STM32 Firmware — `main_fixed_updated.txt` / `stm32_main_c_fix.md`

> **Note:** This firmware plan is documented in `stm32_main_c_fix.md`. The changes below are
> a redesign of the Approach 1 `STM codebase/main.c` — they have **not been applied to the
> `.c` file** in this repository. The `.c` source in this branch remains at the Approach 1 state.
> Apply the changes in `stm32_main_c_fix.md` manually in STM32CubeIDE before flashing.

| # | Status | Change | Detail |
|---|--------|--------|--------|
| 10 | ✅ Designed | **Dual-UART RX** — both UART4 and USART2 accept commands | Adds `uart2_rx` buffer. `proto_uart_start_rx()` arms both UARTs at boot. `HAL_UART_RxCpltCallback` dispatches both to `queue_motion_cmd()`. Makes firmware tolerant to either port being physically connected to Pi. |
| 11 | ✅ Designed | **`proto_uart_tx()` broadcasts ENC to both UARTs** | Single call sends encoder frames to both UART4 and USART2 simultaneously. Encoder frames visible on USB debug without separate serial tap. |
| 12 | ✅ Designed | **Startup hello message removed** | No `DBG,UART4 command link ready\r\n` on boot — eliminates non-ENC noise that triggers bridge warnings at startup. |
| 13 | ✅ Designed | **DBG frames removed from `encoderTask`** | Only clean `ENC,...\r\n` lines sent via `proto_uart_tx()`. No mixed text on UART4. |
| 14 | ✅ Designed | **Watchdog timeout extended: 500 ms → 1000 ms** | Gives the Pi bridge more margin before STM32 triggers emergency stop. Matches bridge watchdog (1 s). |
| 15 | ✅ Designed | **`controlTask` uses 20 ms queue timeout** | `osMessageQueueGet(..., 20)` instead of `osWaitForever` — allows watchdog check to run every 20 ms even when no command arrives. |
| 16 | ✅ Designed | **`applied_cmd` tracking added to `controlTask`** | `apply_cmd()` now only called when `current_cmd != applied_cmd`, preventing redundant PWM writes on every tick. |
| 17 | ❌ Not implemented | **VEL packet parser** | STM32 still only accepts single-byte `F/B/L/R/S`. Parsing `VEL,left,right\r\n` requires: USART line-buffering, `strtok`/`sscanf` parser, per-wheel `__HAL_TIM_SET_COMPARE()` calls, and a second PWM channel — **blocked by hardware Gap G2**. |

---

### Nav2 Parameters — `nav2_params/nav2_params.yaml`

| Parameter | Approach 2 | Approach 1 | Location in file |
|-----------|-----------|------------|-----------------|
| `robot_radius` (local costmap) | 0.22 m | 0.22 m | line 199 |
| `inflation_radius` (local costmap) | **0.40 m** | 0.20 m | line 204 |
| `robot_radius` (global costmap) | **0.25 m** | 0.22 m | line 231 |
| `inflation_radius` (global costmap) | 0.20 m | 0.20 m | line 255 |
| `time_before_collision` | **4.8 s** | 2.0 s | line 386 |

Rationale: larger inflation and longer collision look-ahead compensate for the expected slower
response of the new differential PWM pipeline (higher latency from rate-limiting + unverified
STM32 parser). These values are more conservative and may cause planning failures in narrow spaces.

---

## Relationship to Approach 1

Approach 2 is a **parallel experiment** — it does not replace Approach 1. Both branches exist
independently. Approach 1 (`fix/critical-navigation-and-safety`) is the production-ready approach
using discrete commands. Approach 2 (`Saif`) is a research branch exploring true differential arc
motion, pending the hardware and firmware changes needed to make it functional.
