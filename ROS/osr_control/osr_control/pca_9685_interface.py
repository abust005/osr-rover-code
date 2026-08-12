import rclpy
from rclpy.node import Node
from enum import Enum

# project libraries
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# message imports
from osr_interfaces.msg import CommandPWM

class ChannelType(Enum):
    SERVO = 1
    CONTINUOUS = 2
    GENERIC = 3

class PCA9685Interface(Node):
    def __init__(self):
        super().__init__("pca9685_interface",
                         automatically_declare_parameters_from_overrides=True)
        self.log = self.get_logger()
        self.pca = None

        # Ensure only one PCA9685 interface exists
        node_names = self.get_node_names()
        matching_nodes = [n for n in node_names if n == self.get_name()]
        
        if len(matching_nodes) > 1:
            self.log.error(f"Instance of {self.get_name()} already exists! Shutting down.")
            self.destroy_node()
            return

        # self.connect_pca9685()
        self.channel_params = self.get_parameters_by_prefix('channel_mapping')
        self.params_to_dict()
        self.channel_setup()

        self.cmd_pca_sub = self.create_subscription(CommandPWM, "/cmd_pwm", self.pca9685_cmd_cb, 10)

        # Keep track of last-commanded signal for each channel to avoid commanding the same duty cycle over and over
        self.last_signal = [None] * 16

        # Account for 16 possible servos
        self.servos = [None] * 16

    def connect_pca9685(self):

        self.log.debug("Creating PCA9685 control instance")
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

    def params_to_dict(self):

        map_dict = {}
        for param_key, param_obj in self.channel_params.items():
            item_id, attr = param_key.split('.', 1)
            if item_id not in map_dict:
                map_dict[item_id] = {}
            map_dict[item_id][attr] = param_obj.value

        self.channel_params = map_dict

    def channel_setup(self):

        for params in self.channel_params.values():

            channel = params['channel']
            self.log.info(f"Configuring hardware for channel {channel}")

            ch_type = ChannelType(params['type'])
            if ch_type is ChannelType.GENERIC:
                continue

            range = params['actuation_range']
            pulse_widths = params['pulse_width_range']

            if(ch_type is ChannelType.SERVO):
                self.servos[channel] = servo.Servo(self.pca.channels[channel],
                                                   actuation_range=range,
                                                   min_pulse=pulse_widths[0],
                                                   max_pulse=pulse_widths[1])
            elif(ch_type is ChannelType.CONTINUOUS):
                self.servos[channel] = servo.ContinuousServo(self.pca.channels[channel],
                                                   actuation_range=range,
                                                   min_pulse=pulse_widths[0],
                                                   max_pulse=pulse_widths[1])

    def pca9685_cmd_cb(self, cmd: CommandPWM):

        if not self.pca:
            return

        channel = cmd.channel_id

        if channel < 0 or channel > 15:
            self.log.warn(f"Channel {channel} is out of range, dropping command", throttle_duration_sec=1)
            return

        last_signal = self.last_signal[channel]

        if cmd.duty_cycle != last_signal:

            # handle servos separately from generic PWM peripherals
            if self.servos[channel] != None:
                try:
                    # Use a float cast for angle to ensure precision
                    self.pca.servo[channel].angle = float(cmd.duty_cycle)
                    self.last_signal[channel] = cmd.duty_cycle

                except ValueError:
                    # Handle out of range without killing the node
                    self.log.warn(f"Angle {cmd.duty_cycle} out of range for channel {channel}", throttle_duration_sec=1)
                except Exception as e:
                    self.log.error(f"Servo Error: {e}")
                finally:
                    return

            try:
                self.pca.channels[channel].duty_cycle = cmd.duty_cycle
                self.last_signal[channel] = cmd.duty_cycle
            except Exception as e:
                self.log.error(f"PWM Error: {e}")

def main(args=None):

    rclpy.init(args=args)

    wrapper = PCA9685Interface()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()