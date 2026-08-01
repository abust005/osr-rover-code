import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, TwistWithCovariance
from sensor_msgs.msg import JointState
from osr_interfaces.msg import CommandDrive, Status

import numpy as np

import crossfire

class RcJoy(Node):
  """Node for arbitary radio control using CRSF"""

  def __init__(self):
    super().__init__("rc_joy")
    self.log = self.get_logger()
    self.log.info("Initializing RCJoy Node")

    self.declare_parameters(
      namespace='',
      parameters=[
        ('serial_port', Parameter.Type.STRING),
        ('enable_channel', Parameter.Type.INTEGER),
        ('velocity.channel', Parameter.Type.INTEGER),
        ('velocity.range', Parameter.Type.INTEGER_ARRAY),
        ('velocity.scale', Parameter.Type.DOUBLE),
        ('rotation.channel', Parameter.Type.INTEGER),
        ('rotation.range', Parameter.Type.INTEGER_ARRAY),
        ('rotation.scale', Parameter.Type.DOUBLE),
        ('turbo_channel', Parameter.Type.INTEGER)
      ]
    )

    self.velocity_range = list(self.get_parameter('velocity.range').get_parameter_value().integer_array_value)
    self.rotation_range = list(self.get_parameter('rotation.range').get_parameter_value().integer_array_value)
    
    self.vel_deadzone = (self.velocity_range[2] - self.velocity_range[0]) * 0.1
    self.rot_deadzone = (self.rotation_range[2] - self.rotation_range[0]) * 0.1

    self.log.info(f"Velocity range: {self.velocity_range}")

    self.velocity_transform = lambda x: (x - self.velocity_range[0]) * 2.0 / (self.velocity_range[2] - self.velocity_range[0]) - 1
    self.rotation_transform = lambda x: (x - self.rotation_range[0]) * 2.0 / (self.rotation_range[2] - self.rotation_range[0]) - 1
    
    self.exponential_scale = 1.50

    self.interface = self.get_parameter('serial_port').get_parameter_value().string_value
    self.log.info(f"Got parameter serial_port: {self.interface}")
    self.crsf_port = crossfire.XCrossfire(self.interface)
    self.opened_port = False

    self.port_timer_rate = 1 # second, 1Hz
    self.setup_port_timer = self.create_timer(self.port_timer_rate, self.setup_crossfire)

    self.control_update_rate = 0.01 # seconds, 100Hz
    self.fast_timer = self.create_timer(self.control_update_rate, self.update_control_signal)

    self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

    self.last_channel_rx = []

  def setup_crossfire(self):

    opened_port = self.crsf_port.open_port()

    if not(opened_port):
      self.log.error("CRSF port open failure")
      return

    self.log.info(f"Opened CRSF port on interface {self.interface}")
    self.setup_port_timer.cancel()

  def update_control_signal(self):

    if not(self.crsf_port.is_paired()):
      self.log.error("Transmitter not paired", throttle_duration_sec=5)
      return

    self.last_channel_rx = np.array(self.crsf_port.get_channel_state())

    velocity_value = self.last_channel_rx[self.get_parameter('velocity.channel').get_parameter_value().integer_value - 1]
    rotation_value = self.last_channel_rx[self.get_parameter('rotation.channel').get_parameter_value().integer_value - 1]

    if velocity_value < (self.velocity_range[1] + self.vel_deadzone) and \
        velocity_value > (self.velocity_range[1] - self.vel_deadzone):
      
      velocity_value = 0.0
    
    else:
      velocity_value = self.velocity_transform(velocity_value)
      velocity_value = pow(abs(velocity_value), self.exponential_scale) * (1 if velocity_value >= 0 else -1);
      velocity_value *= self.get_parameter('velocity.scale').get_parameter_value().double_value
      
    if rotation_value < (self.rotation_range[1] + self.rot_deadzone) and \
       rotation_value > (self.rotation_range[1] - self.rot_deadzone):

      rotation_value = 0.0

    else:  
      rotation_value = self.rotation_transform(rotation_value)
      rotation_value = pow(abs(rotation_value), self.exponential_scale) * (1 if rotation_value >= 0 else -1);
      rotation_value *= self.get_parameter('rotation.scale').get_parameter_value().double_value

    msg = Twist()
    msg.linear.x = velocity_value
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = rotation_value

    self.pub.publish(msg)
    # Turbo is applied as a ramp based on the axis value
    # turbo_ramp = turbo_scale_map.at(fieldname) - scale_map.at(fieldname);
    # turbo_gain = (joy_msg->axes[turbo_axis] - 1) * -0.5 * turbo_ramp;
    # return joystick_value * (scale_map.at(fieldname) + turbo_gain);

def main(args=None):
    rclpy.init(args=args)

    wrapper = RcJoy()

    rclpy.spin(wrapper)
    wrapper.stop_motors()
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
