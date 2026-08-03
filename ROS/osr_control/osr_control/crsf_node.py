import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from osr_interfaces.msg import CRSFChannels

import numpy as np

import crossfire

class CRSF_RX(Node):
  """Node for receiving data over CRSF"""

  def __init__(self):
    super().__init__("crsf_rx")
    self.log = self.get_logger()
    self.log.info("Initializing CRSF RX Node")

    self.declare_parameters(
      namespace='',
      parameters=[
        ('serial_device', Parameter.Type.STRING),
        ('baud', Parameter.Type.INTEGER)
      ]
    )
    self.interface = self.get_parameter('serial_device').get_parameter_value().string_value
    self.baud_rate = self.get_parameter('baud').get_parameter_value().integer_value

    self.log.info(f"Got parameter serial_device: {self.interface}")
    self.opened_port = False

    self.port_timer_rate = 1 # second, 1Hz
    self.setup_port_timer = self.create_timer(self.port_timer_rate, self.setup_crossfire)

    self.control_update_rate = 0.01 # seconds, 100Hz
    self.fast_timer = self.create_timer(self.control_update_rate, self.update_control_signal)

    self.crsf_pub = self.create_publisher(CRSFChannels, "/crsf", 1)

  def setup_crossfire(self):

    self.crsf_port = crossfire.XCrossfire(self.interface, self.baud_rate)

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

    msg = CRSFChannels()
    msg.channels = self.crsf_port.get_channel_state()
    self.crsf_pub.publish(msg)
