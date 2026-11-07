import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Joy
from osr_interfaces.msg import CRSFChannels

class RCJoyNode(Node):
    def __init__(self):
        super().__init__('rc_joy')


        self.declare_parameters(
        namespace='',
        parameters=[
            ('enable_channel', Parameter.Type.INTEGER),
            ('velocity.channel', Parameter.Type.INTEGER),
            ('velocity.scale', Parameter.Type.DOUBLE),
            ('rotation.channel', Parameter.Type.INTEGER),
            ('rotation.scale', Parameter.Type.DOUBLE),
            ('turbo_channel', Parameter.Type.INTEGER),
            ('ch_max', Parameter.Type.INTEGER_ARRAY),
            ('ch_mid', Parameter.Type.INTEGER_ARRAY),
            ('ch_min', Parameter.Type.INTEGER_ARRAY),
            ('ch_enable', Parameter.Type.INTEGER_ARRAY),
            ('ch_type', Parameter.Type.INTEGER_ARRAY)
        ]
        )

        # --- Parameters ---
        self.declare_parameter('frame_id', 'rc_transmitter')

        # Deadzone threshold around center (in normalized range 0.0 - 1.0)
        self.declare_parameter('global_deadzone', 0.02)
        self.declare_parameter('ch_deadzones', [-1.0] * 16)

        # --- Read Parameters ---
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.ch_min = self.get_parameter('ch_min').get_parameter_value().integer_array_value
        self.ch_mid = self.get_parameter('ch_mid').get_parameter_value().integer_array_value
        self.ch_max = self.get_parameter('ch_max').get_parameter_value().integer_array_value
        self.ch_enable = self.get_parameter('ch_enable').get_parameter_value().integer_array_value
        self.ch_type = self.get_parameter('ch_type').get_parameter_value().integer_array_value
        self.global_deadzone = self.get_parameter('global_deadzone').get_parameter_value().double_value
        self.ch_deadzones = self.get_parameter('ch_deadzones').get_parameter_value().double_array_value

        for i, d in enumerate(self.ch_deadzones):
            if d < 0:
                self.ch_deadzones[i] = self.global_deadzone

        # --- Publishers and Subscribers ---
        self.joy_pub = self.create_publisher(Joy, '/joy', 1)
        self.crsf_sub = self.create_subscription(
            CRSFChannels,
            '/crsf',
            self.crsf_callback,
            10
        )

        self.get_logger().info(f"RC Joy node listening on /crsf and publishing to /joy")

    def _normalize_axis(self, ch, raw_val: int) -> float:
        """Converts raw PWM/CRSF value (e.g. 988-2012 us) to a [-1.0, 1.0] float range."""

        deadzone = self.ch_deadzones[ch]

        if raw_val >= self.ch_mid[ch]:
            norm = (raw_val - self.ch_mid[ch]) / float(self.ch_max[ch] - self.ch_mid[ch])
        else:
            norm = (raw_val - self.ch_mid[ch]) / float(self.ch_mid[ch] - self.ch_min[ch])

        # Clamp range
        norm = max(-1.0, min(1.0, norm))

        # Deadzone filter
        if abs(norm) < deadzone:
            return 0.0

        return norm

    def crsf_callback(self, msg: CRSFChannels):
        if len(msg.channels) == 0:
            return

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = self.frame_id

        axes = [0.0] * 16
        buttons = [0] * 16

        for ch in range(16):
            ch_val = msg.channels[ch]
            ch_mid = self.ch_mid[ch]

            if not(bool(self.ch_enable[ch])):
                continue

            if self.ch_type[ch] == 0:
                # Populate normalized continuous values into axes
                axes[ch] = self._normalize_axis(ch, ch_val)

            elif self.ch_type[ch] == 1:
                # Populate binary state (0 or 1) into buttons for switch handling
                button_state = 1 if ch_val > ch_mid else 0
                buttons[ch] = button_state

            else:
                button_state = 0

                if ch_val > ch_mid:
                    button_state = 1
                elif ch_val < ch_mid:
                    button_state = -1

                buttons[ch] = button_state

        joy_msg.axes = axes
        joy_msg.buttons = buttons

        self.joy_pub.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RCJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()