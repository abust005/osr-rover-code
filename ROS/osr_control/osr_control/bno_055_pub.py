import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Accel, AccelStamped

import board
import busio
import adafruit_bno055
import yaml
import numpy as np

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        
        # Initialize parameters
        self.declare_parameter('sensor_address', "0x28")
        self.declare_parameter('publish_rate', 2.0)

        # Get parameter values
        self.sensor_address = self.get_parameter('sensor_address').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize BNO055 sensor
        self.get_logger().debug(f"Attempting to connect to BNO055 at address {self.sensor_address} (int: {int(self.sensor_address, 0)})")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno055 = adafruit_bno055.BNO055_I2C(i2c)

        self.calib_data = {}
        # Get calibration data
        with open("/home/bobert/bno055_calib_data2.yaml") as f:
          self.calib_data = yaml.safe_load(f)

        self.bno055.offsets_accelerometer = tuple(self.calib_data["Offsets_Accelerometer"])
        self.bno055.offsets_gyroscope = tuple(self.calib_data["Offsets_Gyroscope"])

        self.angular_accel = 0
        self.last_angular_vel = np.array(self.bno055.gyro)

        # Create publisher
        self.accel_pub = self.create_publisher(Accel, '/accel', QoSProfile(depth=10))

        # Create timer for publishing at the specified rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
        self.fast_timer = self.create_timer(0.01, self.angular_accel_callback)

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f"Started BNO055 node. Publishing at {self.publish_rate} Hz.")

    def angular_accel_callback(self):

        curr_angular_vel = np.array(self.bno055.gyro)
        self.angular_accel = (self.angular_accel + ((curr_angular_vel - self.last_angular_vel) / 0.01)) / 2
        self.last_angular_vel = curr_angular_vel

    def parameter_callback(self, parameters):
        for param in parameters:
            if param.name == 'publish_rate':
                self.publish_rate = param.value
                # Update the timer period with the new publish rate
                self.timer.timer_period = 1.0 / self.publish_rate
                self.timer.cancel()
                # Create a new timer with the updated period
                self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
                self.get_logger().debug(f"Updated timer to run at {self.publish_rate} Hz")
        return SetParametersResult(successful=True, reason="OK")

    def publish_data(self):

        curr_accel = self.bno055.linear_acceleration
        
        accel_msg = AccelStamped()

        accel_msg.header.stamp = self.get_clock().now().to_msg()
        accel_msg.header.frame_id = 'base_link'

        accel_msg.accel.linear.x = curr_accel[0]
        accel_msg.accel.linear.y = curr_accel[1]
        accel_msg.accel.linear.z = curr_accel[2]

        accel_msg.accel.angular.x = self.angular_accel[0]
        accel_msg.accel.angular.y = self.angular_accel[0]
        accel_msg.accel.angular.z = self.angular_accel[0]

        # Publish Accel message
        self.accel_pub.publish(accel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
