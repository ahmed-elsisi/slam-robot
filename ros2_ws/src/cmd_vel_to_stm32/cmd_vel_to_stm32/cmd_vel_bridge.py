import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
import serial

# MPPI limits from nav2_params.yaml
VX_MAX = 0.5
WZ_MAX = 1.9

# "Turn to align, then drive" hysteresis thresholds (normalised by WZ_MAX).
# When both linear and angular are active and ang_norm exceeds ALIGN_TURN_START,
# the bridge commits to turning in place. It resumes driving only after ang_norm
# falls below ALIGN_DRIVE_START. The gap between the two prevents rapid oscillation.
ALIGN_TURN_START  = 0.35   # |wz| / WZ_MAX  ~0.67 rad/s  — enter alignment turn
ALIGN_DRIVE_START = 0.15   # |wz| / WZ_MAX  ~0.29 rad/s  — resume driving

# Counter-brake pulse sent to STM32 when transitioning to stop from F or B.
# STM32 runs the reverse command at full SPEED_DRIVE for this duration, then receives 'S'.
# Reduce BRAKE_PULSE_S if the robot creeps backward; increase if it still coasts forward.
BRAKE_PULSE_S = 0.060


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Set to True via --ros-args -p invert_rotation:=true to swap L/R without recompiling.
        # Use this during physical L/R verification before hardcoding the direction.
        self.declare_parameter('invert_rotation', False)
        self.invert_rotation = (
            self.get_parameter('invert_rotation').get_parameter_value().bool_value
        )

        self.serial_port = '/dev/ttyAMA0'
        self.baudrate = 115200
        self.last_command = None
        self.last_vel_time = self.get_clock().now()
        self.last_motion = None    # last 'F' or 'B' sent — used to pick counter-brake direction
        self.braking = False
        self.brake_timer = None
        self.align_turning = False  # True while doing an in-place alignment turn before driving

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.02)
            self.get_logger().info(
                f'Connected to STM32 on {self.serial_port} at {self.baudrate}'
            )
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Could not open serial port {self.serial_port}: {e}')

        # Subscribe to collision_monitor output (/cmd_vel_out).
        # This is the final safety-limited velocity after the full Nav2 pipeline:
        #   controller_server → /cmd_vel
        #   velocity_smoother → /cmd_vel_smoothed
        #   collision_monitor → /cmd_vel_out   ← we read this
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )

        self.encoder_pub = self.create_publisher(
            Int64MultiArray,
            '/encoder_data',
            10
        )

        self.read_timer = self.create_timer(0.02, self.read_serial)

        # Watchdog: send 'S' to STM32 if no /cmd_vel_out arrives for >1 s.
        # Guards against Nav2 shutdown, bridge restart, or network drop mid-motion.
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

    def _turn_cmd(self, angular_z):
        """Map angular_z sign to 'L'/'R', honouring invert_rotation parameter."""
        left = angular_z > 0
        if self.invert_rotation:
            left = not left
        return 'L' if left else 'R'

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_vel_time).nanoseconds / 1e9
        if elapsed > 1.0 and self.last_command != 'S':
            self.get_logger().warn('No /cmd_vel_out for >1 s — sending stop to STM32')
            self._send('S')

    def _start_brake(self):
        brake_cmd = 'B' if self.last_motion == 'F' else 'F'
        self.get_logger().info(
            f'[brake] counter-pulse {brake_cmd} for {int(BRAKE_PULSE_S * 1000)} ms'
        )
        self._send(brake_cmd)
        self.braking = True
        self.brake_timer = self.create_timer(BRAKE_PULSE_S, self._on_brake_complete)

    def _on_brake_complete(self):
        if self.brake_timer is not None:
            self.brake_timer.cancel()
            self.destroy_timer(self.brake_timer)
            self.brake_timer = None
        self.braking = False
        self.last_motion = None
        self.get_logger().info('[brake] complete -> S')
        self._send('S')
        self.last_command = 'S'

    def cmd_vel_callback(self, msg):
        self.last_vel_time = self.get_clock().now()
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        lin_active = abs(linear_x) > 0.05
        ang_active = abs(angular_z) > 0.05

        if lin_active and ang_active:
            # "Turn to align, then drive" with hysteresis.
            # Enter alignment turn when angular demand is large; exit when it drops enough.
            # Clearing last_motion on entry avoids a spurious brake pulse after a pure spin.
            ang_norm = abs(angular_z) / WZ_MAX
            if not self.align_turning and ang_norm > ALIGN_TURN_START:
                self.align_turning = True
                self.last_motion = None
            elif self.align_turning and ang_norm < ALIGN_DRIVE_START:
                self.align_turning = False
            command = self._turn_cmd(angular_z) if self.align_turning else ('F' if linear_x > 0 else 'B')
        elif lin_active:
            self.align_turning = False
            command = 'F' if linear_x > 0 else 'B'
        elif ang_active:
            self.align_turning = False
            command = self._turn_cmd(angular_z)
        else:
            self.align_turning = False
            command = 'S'

        # If a motion command arrives during braking, cancel the brake immediately.
        # If another S arrives during braking, ignore it — let the timer complete.
        if self.braking:
            if command != 'S':
                if self.brake_timer is not None:
                    self.brake_timer.cancel()
                    self.destroy_timer(self.brake_timer)
                    self.brake_timer = None
                self.braking = False
                self.last_motion = None
            else:
                return

        if command == 'S' and self.last_motion in ('F', 'B'):
            self._start_brake()
            return

        if command != self.last_command:
            self.get_logger().info(
                f'linear_x={linear_x:.2f}, angular_z={angular_z:.2f} -> {command}'
            )
        self._send(command)
        if command in ('F', 'B'):
            self.last_motion = command
        elif command == 'S':
            self.last_motion = None

    def _send(self, command):
        self.last_command = command
        if self.ser is not None:
            try:
                self.ser.write(command.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Failed to send command to STM32: {e}')

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='replace').strip()

                if not line:
                    continue

                if not line.startswith('ENC,'):
                    self.get_logger().warn(f'Ignored non-ENC line: {line}')
                    continue

                parts = line.split(',')

                if len(parts) != 7:
                    self.get_logger().warn(f'Bad ENC format: {line}')
                    continue

                try:
                    seq = int(parts[1])
                    time_ms = int(parts[2])
                    left_delta = int(parts[3])
                    right_delta = int(parts[4])
                    left_total_ticks = int(parts[5])
                    right_total_ticks = int(parts[6])
                except ValueError:
                    self.get_logger().warn(f'Could not parse ENC line: {line}')
                    continue

                enc_msg = Int64MultiArray()
                enc_msg.data = [
                    seq,
                    time_ms,
                    left_delta,
                    right_delta,
                    left_total_ticks,
                    right_total_ticks,
                ]
                self.encoder_pub.publish(enc_msg)

        except Exception as e:
            self.get_logger().error(f'Serial read failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.ser is not None:
        node.ser.write(b'S')
        node.ser.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
