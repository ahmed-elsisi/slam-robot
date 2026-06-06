# Changelog — SLAM Robot — Approach 2 (Saif Branch)
## Differential PWM Arc Motion Attempt

All changes relative to Approach 1 (`fix/critical-navigation-and-safety` branch).
Format: `[Status] | Change | Location | Date`

---

## Critical Known Gaps (HARDWARE — UNRESOLVED)

| # | Issue | Detail |
|---|-------|--------|
| G1 | **Single shared PWM channel** | TIM4 CH1 / PB6 is the only EN pin shared across both L298N drivers — direction per side is independent but speed is averaged `(abs(L) + abs(R)) / 2`. True independent left/right speed control requires a hardware redesign (second PWM channel on second L298N EN). Arc motion produces correct directions but identical speed on both wheels. |

---

## Open Items (inherited from Approach 1, still unresolved)

| # | Item | Notes |
|---|------|-------|
| A | L/R direction not physically confirmed | Use `--ros-args -p invert_rotation:=true` on bridge to test; make permanent if needed |
| B | `slam_virtual_odom.yaml` not tracked in repo — lives on Pi only | Copy from Pi and commit |
| C | Encoder left/right assignment (TIM1=left, TIM8=right) not confirmed | Spin one wheel, check `/encoder_data` fields |
| D | Encoder odometry not fused into `/odom` | Future work: implement differential drive odometry node |

---

## Completed Changes

| # | Status | Change | Date |
|---|--------|--------|------|
| 1 | ✅ Done | **[Protocol] VEL packet protocol** — Pi sends `VEL,left_pwm,right_pwm\r\n`; STM32 line-buffers bytes and parses on `\n`. Replaces single-byte `F/B/L/R/S` commands. | 2026-06-06 |
| 2 | ✅ Done | **[Bridge] `twist_to_pwm()` — differential PWM mixing** — `forward_pwm = f(linear_x)`, `turn_pwm = f(angular_z)`, then `left = forward − turn`, `right = forward + turn`. True arc velocity commands now reach STM32. | 2026-06-06 |
| 3 | ✅ Done | **[Bridge] `enforce_motor_minimums()` — deadband enforcement** — Lifts nonzero sub-stall wheel commands to `MIN_MOVING_PWM=17` (same direction) or `MIN_TURNING_PWM=12` (opposite). Prevents wheel stall on mixed arcs (e.g. L=29, R=5). | 2026-06-06 |
| 4 | ✅ Done | **[Bridge] 10 Hz rate limiter on normal velocity packets** — `CMD_SEND_INTERVAL_S=0.10`. Stop commands bypass the limiter and are sent immediately. Prevents serial flooding. | 2026-06-06 |
| 5 | ✅ Done | **[Bridge] Watchdog sends `VEL,0,0` (not `'S'`)** — If no `/cmd_vel_out` for >1 s, watchdog uses the new packet format. | 2026-06-06 |
| 6 | ✅ Done | **[Bridge] Non-ENC log suppression** — Batched, max one warning per 5 s window. Prevents log spam from STM32 boot noise. | 2026-06-06 |
| 7 | ✅ Done | **[STM32] VEL packet parser** — `parse_uart_byte()` line-buffers incoming bytes; on `\n`, calls `sscanf` to extract `left_pwm`/`right_pwm`; clamps to `±MAX_PWM=49`; pushes to `velQueueHandle`. | 2026-06-06 |
| 8 | ✅ Done | **[STM32] `apply_velocity(left_pwm, right_pwm)`** — Left and right GPIO direction set independently. Speed = `(abs(L) + abs(R)) / 2` on shared TIM4 CH1 (hardware constraint). Replaces `apply_cmd('F'/'B'/'L'/'R'/'S')`. | 2026-06-06 |
| 9 | ✅ Done | **[STM32] Dual-UART command RX** — Both UART4 and USART2 accept VEL packets. USART2 RX is armed but callback is receive-only (does not control motion), keeping debug port safe. | 2026-06-06 |
| 10 | ✅ Done | **[STM32] Startup hello message removed** — No `DBG,...` on boot; eliminates non-ENC noise at startup that triggered bridge warnings. | 2026-06-06 |
| 11 | ✅ Done | **[STM32] DBG frames removed from encoderTask** — Only clean `ENC,...\r\n` frames sent via `proto_uart_tx()` (broadcast to both UART4 and USART2). | 2026-06-06 |
| 12 | ✅ Done | **[STM32] Watchdog timeout 500 ms → 1000 ms** — Matches Pi bridge watchdog (1 s). `defaultTask` calls `apply_velocity(0, 0)` on timeout. | 2026-06-06 |
| 13 | ✅ Done | **[Nav2] `inflation_radius` 0.20 → 0.40 m (local costmap)** — More conservative inflation to compensate for expected arc planning with differential drive. | 2026-06-06 |
| 14 | ✅ Done | **[Nav2] `robot_radius` 0.22 → 0.25 m (global costmap)** — Slightly larger footprint in global planner for conservative path planning. | 2026-06-06 |
| 15 | ✅ Done | **[Nav2] `time_before_collision` 2.0 → 4.8 s** — Extended look-ahead to account for arc motion taking longer to resolve than straight-line moves. | 2026-06-06 |
