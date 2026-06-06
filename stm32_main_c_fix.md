# Corrected `main.c` plan for `slam-robot`

This is a drop-in edit plan for `STM codebase/main.c` from branch
`fix/critical-navigation-and-safety`.

## Why this fixes the current failure

- ROS `cmd_vel_bridge` sends one-byte commands: `F`, `B`, `L`, `R`, `S`.
- ROS expects only encoder frames back, in the form:
  `ENC,<seq>,<time_ms>,<left_delta>,<right_delta>,<left_total>,<right_total>`
- The current firmware mixes protocol traffic and debug text across STM UARTs.
- The runtime logs show the Pi is not receiving clean `ENC,...` frames and the car
  is not reacting to drive commands.

The safest firmware fix is:

- accept commands on both `UART4` and `USART2`
- emit clean encoder frames on both `UART4` and `USART2`
- remove all debug text from those UARTs

That makes the firmware tolerant to the actual wired port while preserving the
existing ROS protocol.

## Replace/add these sections in `main.c`

### 1. Private variables

Add a second RX byte for `USART2` and small UART helpers.

```c
volatile uint8_t uart4_rx;
volatile uint8_t uart2_rx;

osMessageQueueId_t cmdQueueHandle;
volatile uint32_t last_cmd_time_ms = 0;

typedef struct
{
  char cmd;
} CmdMsg_t;

static inline void proto_uart_start_rx(UART_HandleTypeDef *huart, volatile uint8_t *rx_byte)
{
  HAL_UART_Receive_IT(huart, (uint8_t *)rx_byte, 1);
}

static inline void proto_uart_rearm(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart4_rx, 1);
  }
  else if (huart == &huart2)
  {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_rx, 1);
  }
}

static inline void queue_motion_cmd(uint8_t rx)
{
  char c = (rx >= 'a' && rx <= 'z') ? (rx - 'a' + 'A') : rx;

  if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S')
  {
    CmdMsg_t msg;
    msg.cmd = c;
    last_cmd_time_ms = HAL_GetTick();
    osMessageQueuePut(cmdQueueHandle, &msg, 0, 0);
  }
}

static inline void proto_uart_tx(const uint8_t *buf, uint16_t len)
{
  HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 20);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 20);
}
```

### 2. Boot section in `main()`

Delete the startup debug transmit completely, then start RX on both UARTs.

Replace this behavior:

```c
const char hello_uart4[] = "DBG,UART4 command link ready\r\n";
HAL_UART_Transmit(&huart2, (uint8_t*)hello_uart4, strlen(hello_uart4), HAL_MAX_DELAY);
HAL_UART_Transmit(&huart4, (uint8_t*)hello_uart4, strlen(hello_uart4), HAL_MAX_DELAY);
```

with nothing.

Then replace:

```c
HAL_UART_Receive_IT(&huart4, (uint8_t*)&uart4_rx, 1);
```

with:

```c
proto_uart_start_rx(&huart4, &uart4_rx);
proto_uart_start_rx(&huart2, &uart2_rx);
```

### 3. `HAL_UART_RxCpltCallback`

Replace the callback body with this:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    queue_motion_cmd(uart4_rx);
    proto_uart_rearm(&huart4);
  }
  else if (huart == &huart2)
  {
    queue_motion_cmd(uart2_rx);
    proto_uart_rearm(&huart2);
  }
}
```

### 4. `HAL_UART_ErrorCallback`

Use the same re-arm logic for both protocol UARTs.

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4 || huart == &huart2)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    proto_uart_rearm(huart);
  }
}
```

### 5. `StartControlTask`

Remove the `CMD:` debug print. Keep only command application and watchdog logic.

Use this structure:

```c
void StartControlTask(void *argument)
{
  CmdMsg_t msg;

  for (;;)
  {
    if (osMessageQueueGet(cmdQueueHandle, &msg, NULL, 20) == osOK)
    {
      current_cmd = msg.cmd;
    }

    if ((HAL_GetTick() - last_cmd_time_ms) > 1000U)
    {
      current_cmd = 'S';
    }

    if (current_cmd != applied_cmd)
    {
      apply_cmd(current_cmd);
      applied_cmd = current_cmd;
    }

    osDelay(20);
  }
}
```

### 6. `StartEncoderTask`

Keep the encoder math, but send only clean `ENC,...` lines through `proto_uart_tx()`.
Remove any `DBG,...` transmit.

The transmit part should look like this:

```c
int tx_len = snprintf(
  tx_buf,
  sizeof(tx_buf),
  "ENC,%lu,%lu,%ld,%ld,%ld,%ld\r\n",
  (unsigned long)seq,
  (unsigned long)HAL_GetTick(),
  (long)left_delta,
  (long)right_delta,
  (long)left_total_ticks,
  (long)right_total_ticks
);

if (tx_len > 0)
{
  proto_uart_tx((const uint8_t *)tx_buf, (uint16_t)tx_len);
}
```

If there is any existing block that prints lines starting with `DBG,`, delete it.

## Resulting behavior

- If the Pi is physically wired to `UART4`, the robot accepts commands and returns
  clean encoder frames.
- If the Pi is physically wired to `USART2`, the robot also works.
- No debug text corrupts the ROS serial parser.

## What to verify after flashing

1. Start the bridge and confirm the `Ignored non-ENC` warnings disappear.
2. Confirm encoder traffic appears on the Pi:
   `ENC,<...>`
3. Send a manual command and verify motion:
   `ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist ...`
4. Confirm the bridge starts publishing `/encoder_data`.
