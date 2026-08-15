import rclpy
import time
from enum import Enum
from rclpy.node import Node

import sensors

# message imports
from osr_interfaces.msg import CommandPWM, Status
from sensor_msgs.msg import JointState, Joy

class FanSpeed(Enum):
    SHUTDOWN = 0
    IDLE = 25
    LOW = 50
    MID = 75
    HIGH = 100

class Override(Enum):
    DISABLED = 2
    POSITIVE = 1
    NEGATIVE = 0

class ChassisFan(Node):
    def __init__(self):
        super().__init__("chassis_fan")
        self.log = self.get_logger()

        self.fan_channel = 4

        self.pwm_cmd = CommandPWM()
        self.pwm_cmd.channel = self.fan_channel
        
        self.cmd_pwm_pub = self.create_publisher(CommandPWM, "/cmd_pwm", 1)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_cb, 1)

        sensors.init()

        slow_loop_rate = 5  # seconds
        self.slow_timer = self.create_timer(slow_loop_rate, self.fan_control)

        self.last_speed = FanSpeed.SHUTDOWN
        self.override = Override.DISABLED

    def __del__(self):

        self.pwm_cmd.duty_cycle = FanSpeed.SHUTDOWN.value
        self.cmd_pwm_pub.publish(self.pwm_cmd)

    def joy_cb(self, msg: Joy):

        override_ch = msg.buttons[7]

        if override_ch < 0:
            self.override = Override.POSITIVE
        elif override_ch > 1:
            self.override = Override.NEGATIVE
        else:
            self.override = Override.DISABLED
        
    def fan_control(self):

        curr_temp = 0

        for chip in sensors.iter_detected_chips():
            for feature in chip:
                if "temp" in feature.label:
                    curr_temp = float(feature.get_value())

        self.log.info(f"Current CPU temperature at: {curr_temp}")

        fan_speed = FanSpeed.IDLE

        if curr_temp > 50:
            fan_speed = FanSpeed.LOW
        if curr_temp > 60:
            fan_speed = FanSpeed.MID
        if curr_temp > 70:
            fan_speed = FanSpeed.HIGH

        if self.override == Override.POSITIVE:
            fan_speed = FanSpeed.HIGH
        elif self.override == Override.NEGATIVE:
            fan_speed = FanSpeed.SHUTDOWN

        if fan_speed != self.last_speed:

            self.log.info(f"Fan Speed Update: {fan_speed.name}")

            self.pwm_cmd.duty_cycle = float(fan_speed.value)
            self.cmd_pwm_pub.publish(self.pwm_cmd)

            self.last_speed = fan_speed


def main(args=None):
    rclpy.init(args=args)

    node = ChassisFan()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
