import rclpy
import time
from enum import Enum
from rclpy.node import Node
from rclpy.parameter import Parameter

# message imports
from sensor_msgs.msg import Joy
from osr_interfaces.msg import CommandServoKit, Status

class ColorPwm(Enum):
    RED = 42
    GREEN = 85

class StatusLight(Node):
    def __init__(self):
        super().__init__("status_light")
        self.log = self.get_logger()

        self.status_light_channel = 7
        self.light_range = 300
        self.pulse_width_range = (500,2500)

        self.servokit_cmd = CommandServoKit()
        self.servokit_cmd.src = "status_light"
        
        self.joy_sub = self.create_subscription(Joy, "/joy", self.drive_state_cb, 1)
        self.cmd_servokit_pub = self.create_publisher(CommandServoKit, "/cmd_servokit", 1)

        self.setup_light()

        self.was_enabled = False 

        self.color = ColorPwm.RED

    def setup_light(self):

        self.servokit_cmd.setup = True
        self.servokit_cmd.servo_id = self.status_light_channel
        self.servokit_cmd.actuation_range = self.light_range
        self.servokit_cmd.pulse_width_range = [*self.pulse_width_range]
        self.servokit_cmd.new_angle = 42
        
        self.log.info(f"Setting up light on channel {self.status_light_channel}")

        self.cmd_servokit_pub.publish(self.servokit_cmd)

        time.sleep(0.1)

        self.servokit_cmd.setup = False
        self.cmd_servokit_pub.publish(self.servokit_cmd)

    def drive_state_cb(self, msg: Joy):

        is_enabled = bool(msg.buttons[4])

        if is_enabled:
            self.color = ColorPwm.GREEN
        else:
            self.color = ColorPwm.RED

        self.servokit_cmd.new_angle = self.color.value 

        if is_enabled != self.was_enabled:
            
            self.log.info(f"Enable updated to {is_enabled}, publishing status light command with color {self.color.name}")

            self.servokit_cmd.setup = False
            self.cmd_servokit_pub.publish(self.servokit_cmd)

        self.was_enabled = is_enabled

def main(args=None):
    rclpy.init(args=args)

    node = StatusLight()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
