import rclpy
import time
from enum import Enum
from rclpy.node import Node
from rclpy.parameter import Parameter

# message imports
from sensor_msgs.msg import Joy
from osr_interfaces.msg import CommandPWM, Status

class ColorPwm(Enum):
    RED = 28
    GREEN = 50

class StatusLight(Node):
    def __init__(self):
        super().__init__("status_light")
        self.log = self.get_logger()

        self.status_light_channel = 7

        self.pwm_cmd = CommandPWM()
        self.pwm_cmd.src = "status_light"
        
        self.joy_sub = self.create_subscription(Joy, "/joy", self.drive_state_cb, 1)
        self.cmd_pwm_pub = self.create_publisher(CommandPWM, "/cmd_pwm", 1)

        self.was_enabled = False 

        self.color = ColorPwm.RED

    def drive_state_cb(self, msg: Joy):

        is_enabled = bool(msg.buttons[4])

        if is_enabled:
            self.color = ColorPwm.GREEN
        else:
            self.color = ColorPwm.RED

        self.pwm_cmd.duty_cycle = float(self.color.value) 

        if is_enabled != self.was_enabled:
            
            self.log.info(f"Enable updated to {is_enabled}, publishing status light command with color {self.color.name}")

            self.cmd_pwm_pub.publish(self.pwm_cmd)

        self.was_enabled = is_enabled

def main(args=None):
    rclpy.init(args=args)

    node = StatusLight()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
