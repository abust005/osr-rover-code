import rclpy
import time
from enum import Enum
from rclpy.node import Node

import sensors

# message imports
from osr_interfaces.msg import CommandServoKit, Status

class FanSpeed(Enum):
    SHUTDOWN = 0
    IDLE = 25
    LOW = 50
    MID = 75
    HIGH = 99

class ChassisFan(Node):
    def __init__(self):
        super().__init__("chassis_fan")
        self.log = self.get_logger()

        self.fan_channel = 4
        self.fan_range = 100
        self.pulse_width_range = (0,20000)

        self.servokit_cmd = CommandServoKit()
        self.servokit_cmd.src = "chassis_fan"
        
        self.cmd_servokit_pub = self.create_publisher(CommandServoKit, "/cmd_servokit", 1)

        self.setup_fan()

        slow_loop_rate = 5  # seconds
        self.slow_timer = self.create_timer(slow_loop_rate, self.fan_control)

        self.last_speed = FanSpeed.SHUTDOWN

    def __del__(self):

        self.servokit_cmd.setup = False
        self.servokit_cmd.new_angle = FanSpeed.SHUTDOWN.value
        self.cmd_servokit_pub.publish(self.servokit_cmd)

    def setup_fan(self):

        sensors.init()

        self.servokit_cmd.setup = True
        self.servokit_cmd.channel_id = self.fan_channel
        self.servokit_cmd.actuation_range = self.fan_range
        self.servokit_cmd.pulse_width_range = [*self.pulse_width_range]
        
        self.cmd_servokit_pub.publish(self.servokit_cmd)

        self.servokit_cmd.setup = False

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

        if fan_speed != self.last_speed:

            self.log.info(f"Fan Speed Update: {fan_speed.name}")

            self.servokit_cmd.setup = False
            self.servokit_cmd.new_angle = fan_speed.value
            self.cmd_servokit_pub.publish(self.servokit_cmd)

            self.last_speed = fan_speed


def main(args=None):
    rclpy.init(args=args)

    node = ChassisFan()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
