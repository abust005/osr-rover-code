import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# project libraries
from adafruit_servokit import ServoKit

# message imports
from osr_interfaces.msg import CommandServoKit, Status

class ServokitInterface(Node):
    def __init__(self):
        super().__init__("servokit_interface")
        self.log = self.get_logger()
        self.kit = None

        self.connect_pca9685()

        self.cmd_servokit_sub = self.create_subscription(CommandServoKit, "/cmd_servokit", self.servokit_cmd_cb, 10)


    def connect_pca9685(self):
        self.log.debug("Creating ServoKit instance")
        self.kit = ServoKit(channels=16)

    def servokit_cmd_cb(self, cmd: CommandServoKit):
        if not self.kit:
            return

        channel = cmd.servo_id

        # ONLY update hardware config if the setup flag is true
        if cmd.setup:
            self.log.info(f"Configuring hardware for channel {channel}")
            try:
                self.kit.servo[channel].actuation_range = int(cmd.actuation_range)
                self.kit.servo[channel].set_pulse_width_range(*cmd.pulse_width_range)
            except Exception as e:
                self.log.error(f"Setup failed: {e}")
            return

        # LEAN path for high-frequency movement commands
        if cmd.new_angle is not None:
            try:
                # Use a float cast for angle to ensure precision
                self.kit.servo[channel].angle = float(cmd.new_angle)
            except ValueError:
                # Handle out of range without killing the node
                self.log.warn(f"Angle {cmd.new_angle} out of range for channel {channel}", throttle_duration_sec=1)
            except Exception as e:
                self.log.error(f"Servo Error: {e}")
def main(args=None):
    rclpy.init(args=args)

    wrapper = ServokitInterface()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()