import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
import serial
import time


# ============================================================
# Robot / STM32 velocity mapping constants
# Tune these values for your physical robot.
# ============================================================

MAX_LINEAR_MPS = 0.35     # linear.x value that maps to full PWM
MAX_ANGULAR_RPS = 0.80    # angular.z value that maps to full turn PWM
MAX_PWM = 35              # Must match STM32 TIM4 Period
MIN_MOVING_PWM = 17       # Minimum nonzero PWM needed to move forward or backward
MIN_TURNING_PWM = 12      # Minimum nonzero PWM needed to turn left or right
MAX_TURNING_PWM = 24      # Maximum PWM used for turning only
CMD_SEND_INTERVAL_S = 0.10  # Limit normal velocity packets to 10 Hz

DEADZONE_LINEAR = 0.02    # Ignore very small linear velocity commands
DEADZONE_ANGULAR = 0.05   # Ignore very small angular velocity commands


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # If the robot rotates opposite to the expected direction,
        # run with:
        # ros2 run <package_name> cmd_vel_bridge --ros-args -p invert_rotation:=true
        self.declare_parameter('invert_rotation', False)
        self.invert_rotation = (
            self.get_parameter('invert_rotation').get_parameter_value().bool_value
        )

        self.serial_port = '/dev/ttyAMA0'
        self.baudrate = 115200

        self.ser = None
        self.last_vel_time = self.get_clock().now()
        self.last_cmd_send_time = None
        self._last_sent_pwm = (0, 0)

        # Used only for reducing repeated log messages.
        # Packets are still sent repeatedly to keep the STM32 watchdog alive.
        self._last_logged_pkt = None

        # Serial read diagnostics
        self.non_enc_count = 0
        self.non_enc_example = ''
        self.non_enc_last_log_s = 0.0

        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.02,
                write_timeout=0.02
            )
            self.get_logger().info(
                f'Connected to STM32 on {self.serial_port} at {self.baudrate}'
            )
        except Exception as e:
            self.ser = None
            self.get_logger().error(
                f'Could not open serial port {self.serial_port}: {e}'
            )

        # Subscribe to collision_monitor output.
        # This should be the final safety-limited Nav2 velocity:
        #
        # controller_server -> /cmd_vel
        # velocity_smoother -> /cmd_vel_smoothed
        # collision_monitor -> /cmd_vel_out
        #
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

        # Read encoder packets from STM32 at 50 Hz
        self.read_timer = self.create_timer(0.02, self.read_serial)

        # If Nav2 stops publishing, send VEL,0,0 repeatedly.
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

    def twist_to_pwm(self, linear_x, angular_z):
        """
        Convert ROS Twist command into left/right PWM values.

        ROS convention:
        - positive linear_x  = forward
        - negative linear_x  = backward
        - positive angular_z = rotate left / counter-clockwise

        Differential PWM mixing:
        left_pwm  = forward_pwm - turn_pwm
        right_pwm = forward_pwm + turn_pwm
        """

        if abs(linear_x) < DEADZONE_LINEAR:
            linear_x = 0.0

        if abs(angular_z) < DEADZONE_ANGULAR:
            angular_z = 0.0

        if self.invert_rotation:
            angular_z = -angular_z

        def velocity_to_pwm(value, max_value, min_pwm, max_pwm):
            if value == 0.0:
                return 0

            pwm = int((value / max_value) * max_pwm)

            if pwm == 0 or abs(pwm) < min_pwm:
                return min_pwm if value > 0 else -min_pwm

            return pwm

        forward_pwm = velocity_to_pwm(linear_x, MAX_LINEAR_MPS, MIN_MOVING_PWM, MAX_PWM)
        turn_pwm = velocity_to_pwm(angular_z, MAX_ANGULAR_RPS, MIN_TURNING_PWM, MAX_TURNING_PWM)

        left_pwm = forward_pwm - turn_pwm
        right_pwm = forward_pwm + turn_pwm

        left_pwm, right_pwm = self.enforce_motor_minimums(left_pwm, right_pwm)

        # Clamp for safety
        left_pwm = max(-MAX_PWM, min(MAX_PWM, left_pwm))
        right_pwm = max(-MAX_PWM, min(MAX_PWM, right_pwm))

        return left_pwm, right_pwm

    def enforce_motor_minimums(self, left_pwm, right_pwm):
        """
        Keep mixed curve commands above the physical motor deadband.

        The forward/turn mix can create values like L=29, R=5. That right
        wheel command is nonzero but too small to move the robot, so forward
        arcs and reverse arcs fail. Preserve zero stops, but lift nonzero
        wheel commands to the minimum PWM needed for that movement type.
        """

        if left_pwm == 0 and right_pwm == 0:
            return 0, 0

        same_direction = (
            left_pwm != 0 and
            right_pwm != 0 and
            (left_pwm > 0) == (right_pwm > 0)
        )
        min_pwm = MIN_MOVING_PWM if same_direction else MIN_TURNING_PWM

        def apply_minimum(pwm):
            if pwm == 0 or abs(pwm) >= min_pwm:
                return pwm
            return min_pwm if pwm > 0 else -min_pwm

        return apply_minimum(left_pwm), apply_minimum(right_pwm)

    def cmd_vel_callback(self, msg):
        now = self.get_clock().now()
        self.last_vel_time = now

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left_pwm, right_pwm = self.twist_to_pwm(linear_x, angular_z)

        is_stop = left_pwm == 0 and right_pwm == 0
        should_send_stop = is_stop and self._last_sent_pwm != (0, 0)

        if self.last_cmd_send_time is not None:
            elapsed = (now - self.last_cmd_send_time).nanoseconds / 1e9
            if elapsed < CMD_SEND_INTERVAL_S and not should_send_stop:
                return

        self.last_cmd_send_time = now
        self._last_sent_pwm = (left_pwm, right_pwm)

        self.send_velocity_packet(
            left_pwm,
            right_pwm,
            log_text=(
                f'linear={linear_x:.2f}, angular={angular_z:.2f} '
                f'-> L={left_pwm}, R={right_pwm}'
            )
        )

    def send_velocity_packet(self, left_pwm, right_pwm, log_text=None):
        """
        Send one velocity packet to STM32.

        Packet format:
        VEL,left_pwm,right_pwm\\r\\n

        Example:
        VEL,25,18
        VEL,0,0
        VEL,-20,20
        """

        left_pwm = max(-MAX_PWM, min(MAX_PWM, int(left_pwm)))
        right_pwm = max(-MAX_PWM, min(MAX_PWM, int(right_pwm)))

        pkt = f'VEL,{left_pwm},{right_pwm}\r\n'

        # Suppress identical log messages only.
        # Do not suppress the actual serial write, because STM32 watchdog
        # should keep receiving valid packets.
        if pkt != self._last_logged_pkt:
            if log_text is None:
                self.get_logger().info(f'Sending {pkt.strip()}')
            else:
                self.get_logger().info(log_text)

            self._last_logged_pkt = pkt

        if self.ser is not None:
            try:
                self.ser.write(pkt.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_vel_time).nanoseconds / 1e9

        if elapsed > 1.0:
            self.get_logger().warn(
                'No /cmd_vel_out for >1 s — sending VEL,0,0 to STM32'
            )
            self.send_velocity_packet(0, 0)

    def _handle_non_enc_line(self, line):
        self.non_enc_count += 1
        self.non_enc_example = line

        now_s = time.monotonic()

        if now_s - self.non_enc_last_log_s < 5.0:
            return

        self.get_logger().warn(
            'Ignoring non-ENC serial data from STM32; '
            f'suppressed {self.non_enc_count} line(s) in the last window. '
            f'Latest: {self.non_enc_example}'
        )

        self.non_enc_count = 0
        self.non_enc_last_log_s = now_s

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='replace').strip()

                if not line:
                    continue

                if not line.startswith('ENC,'):
                    self._handle_non_enc_line(line)
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
        try:
            node.ser.write(b'VEL,0,0\r\n')
            node.ser.close()
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
