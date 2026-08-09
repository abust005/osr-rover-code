import rclpy
from rclpy.node import Node

# project libraries
from adafruit_servokit import ServoKit

# message imports
from osr_interfaces.msg import CommandServoKit

class ServokitInterface(Node):
    def __init__(self):
        super().__init__("servokit_interface")
        self.log = self.get_logger()
        self.kit = None

        # Ensure only one servokit interface exists
        node_names = self.get_node_names()
        matching_nodes = [n for n in node_names if n == self.get_name()]
        
        if len(matching_nodes) > 1:
            self.log.error(f"Instance of {self.get_name()} already exists! Shutting down.")
            self.destroy_node()

        self.connect_pca9685()

        self.cmd_servokit_sub = self.create_subscription(CommandServoKit, "/cmd_servokit", self.servokit_cmd_cb, 10)

    def connect_pca9685(self):
        self.log.debug("Creating ServoKit instance")
        self.kit = ServoKit(channels=16)

    def servokit_cmd_cb(self, cmd: CommandServoKit):
        if not self.kit:
            return

        channel = cmd.servo_id

        if channel < 0 or channel > 15:
            self.log.warn(f"Channel {channel} is out of range, dropping command", throttle_duration_sec=1)
            return

        # ONLY update hardware config if the setup flag is true
        if cmd.setup:
            self.log.info(f"Configuring hardware for channel {channel}")
            try:
                self.kit.servo[channel].actuation_range = int(cmd.actuation_range)
                self.kit.servo[channel].set_pulse_width_range(*cmd.pulse_width_range)
            except Exception as e:
                self.log.error(f"Setup failed: {e}")

        # LEAN path for high-frequency movement commands
        if cmd.new_angle != 0:
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