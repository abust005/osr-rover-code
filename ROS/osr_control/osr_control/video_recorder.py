import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros

import cv2
import depthai as dai

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class VideoRecorder(Node):
  def __init__(self):
    # Create pipeline
    self.pipeline = dai.Pipeline()

    # Define source and output
    self.camRgb = self.pipeline.create(dai.node.ColorCamera)
    self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)

    self.xoutVideo.setStreamName("video")

    # Properties
    self.camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    self.camRgb.setFps(20)
    self.camRgb.setVideoSize(1920, 1080)

    self.xoutVideo.input.setBlocking(False)
    self.xoutVideo.input.setQueueSize(1)

    # Linking
    self.camRgb.video.link(self.xoutVideo.input)

    self.device = dai.Device(self.pipeline)