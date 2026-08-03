import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, TwistWithCovariance
from sensor_msgs.msg import JointState, Joy
from osr_interfaces.msg import CommandDrive, CRSFChannels, Status

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from osr_interfaces.msg import CRSFChannels


class RCJoyNode(Node):
    def __init__(self):
        super().__init__('rc_joy_node')


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
        self.declare_parameter('deadzone', 0.02)

        # --- Read Parameters ---
        self.frame_id = self.get_parameter('frame_id').value
        self.ch_min = self.get_parameter_value('ch_min').integer_array_value
        self.ch_mid = self.get_parameter_value('ch_mid').integer_array_value
        self.ch_max = self.get_parameter_value('ch_max').integer_array_value
        self.ch_enable = self.get_parameter_value('ch_enable').integer_array_value
        self.ch_type = self.get_parameter_value('ch_type').integer_array_value
        self.deadzone = self.get_parameter('deadzone').double_value

        # --- Publishers and Subscribers ---
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.crsf_sub = self.create_subscription(
            CRSFChannels,
            '/crsf',
            self.crsf_callback,
            10
        )

        self.get_logger().info(f"RC Joy node listening on /crsf and publishing to /joy")

    def _normalize_axis(self, ch, raw_val: int) -> float:
        """Converts raw PWM/CRSF value (e.g. 988-2012 us) to a [-1.0, 1.0] float range."""
        if raw_val >= self.ch_mid[ch]:
            norm = (raw_val - self.ch_mid[ch]) / float(self.ch_max[ch] - self.ch_mid[ch])
        else:
            norm = (raw_val - self.ch_mid[ch]) / float(self.ch_mid[ch] - self.ch_min[ch])

        # Clamp range
        norm = max(-1.0, min(1.0, norm))

        # Deadzone filter
        if abs(norm) < self.deadzone:
            return 0.0

        return norm

    def crsf_callback(self, msg: CRSFChannels):
        if not msg.channels:
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