import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
import serial


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.serial_port = '/dev/ttyAMA0'
        self.baudrate = 115200

        self.last_command = None

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.02)
            self.get_logger().info(f'Connected to STM32 on {self.serial_port} at {self.baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Could not open serial port {self.serial_port}: {e}')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.encoder_pub = self.create_publisher(
            Int64MultiArray,
            '/encoder_data',
            10
        )

        self.read_timer = self.create_timer(0.02, self.read_serial)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0.05:
            command = 'F'
        elif linear_x < -0.05:
            command = 'B'
        elif angular_z > 0.05:
            command = 'L'
        elif angular_z < -0.05:
            command = 'R'
        else:
            command = 'S'

        if command != self.last_command:
            self.get_logger().info(
                f'linear_x={linear_x:.2f}, angular_z={angular_z:.2f} -> {command}'
            )
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

                msg = Int64MultiArray()
                msg.data = [
                    seq,
                    time_ms,
                    left_delta,
                    right_delta,
                    left_total_ticks,
                    right_total_ticks
                ]

                self.encoder_pub.publish(msg)

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
