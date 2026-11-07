import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup
from osr_interfaces.msg import CommandMast

from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class JoyButtonSubscriber(Node):
    """For custom functionality that the joy to twist publisher node can't handle"""
    def __init__(self):
        super().__init__('joy_button_subscriber')
        self.log = self.get_logger()
        self.declare_parameter('duty_button_index', 0)  # Default button index is 0
        self.duty_button_index = self.get_parameter('duty_button_index').value
        self.last_duty_mode_value = False
        self.last_received_time = None        
        self.cb_group = ReentrantCallbackGroup()

        self.teleop_joy_client = self.create_client(SetParameters, '/teleop_twist_joy_node/set_parameters', 
                                                                 callback_group=self.cb_group)
        while not self.teleop_joy_client.wait_for_service(timeout_sec=5.0):
            self.log.info('Service /teleop_twist_joy_node/set_parameters not available, waiting again...', skip_first=True)
        
        self.subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            5)

        self.mast_pub = self.create_publisher(
            CommandMast,
            "/cmd_mast",
            1
        )

        self.control_update_rate = 0.5 # seconds, 20Hz
        self.client_timer = self.create_timer(self.control_update_rate, self.update_lin_ang_vel)

        self.last_x_scale = 1.0
        self.last_ang_scale = -0.75

        self.log.info("Initialized JoyButtonSubscriber.")

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= self.duty_button_index:
            error_msg = f"button index {self.duty_button_index} exceeds the number of buttons available ({len(msg.buttons)})"
            self.log.error(error_msg)
            raise AttributeError(error_msg)

        # Construct CommandMast
        mast_msg = CommandMast()
        mast_msg.azimuth = msg.axes[10] * 1.57
        mast_msg.elevation = msg.axes[11] * 1.57

        self.mast_pub.publish(mast_msg)

        # Channels 12-13 for vel. adj.
        new_x_scale = msg.axes[12]
        new_ang_scale = msg.axes[13]

        # scaling [a,b]
        # x' = a + ((x-min(x)) * (b-a))/ (max(x) - min(x))

        # scale between 1 m/s and 4 m/s
        new_x_scale = 1 + 3 * ((new_x_scale + 1) / 2)

        # scale between 0.75 rad/s and 2.5 rad/s
        new_ang_scale = 0.75 + (((new_ang_scale + 1) * 1.75)/ 2)

        # Clamp range
        self.last_x_scale = max(1.0, min(4.0, new_x_scale))
        self.last_ang_scale = max(0.750, min(2.50, new_ang_scale)) * -1.0

    def update_lin_ang_vel(self):
        req = SetParameters.Request()
        
        # Define the parameter change
        param = Parameter()
        param.name = 'scale_linear.x'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(self.last_x_scale)
        
        req.parameters.append(param)

        param = Parameter()
        param.name = 'scale_angular.yaw'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(self.last_ang_scale)
        
        self.teleop_joy_client.call_async(req)
        self.get_logger().info(f'Requested scale_linear.x change to {self.last_x_scale}, scale_angular.yaw to {self.last_ang_scale}')

def main(args=None):
    rclpy.init(args=args)
    joy_button_subscriber = JoyButtonSubscriber()
    rclpy.spin(joy_button_subscriber)
    joy_button_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
