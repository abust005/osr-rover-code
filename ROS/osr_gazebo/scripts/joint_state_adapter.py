#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateTransformer(Node):
    def __init__(self):
        super().__init__('joint_state_transformer')
        
        self.name_mapping = {
            'front_wheel_joint_L': 'corner_left_front',
            'front_wheel_joint_R': 'corner_right_front',
            'rear_wheel_joint_L': 'corner_left_back',
            'rear_wheel_joint_R': 'corner_right_back',
            'front_wheel_joint_left': 'drive_left_front',
            'front_wheel_joint_right': 'drive_right_front',
            'middle_wheel_joint_left': 'drive_left_middle',
            'middle_wheel_joint_right': 'drive_right_middle',
            'rear_wheel_joint_left': 'drive_left_back',
            'rear_wheel_joint_right': 'drive_right_back'
        }
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        # Publish to BOTH topics that the rover node expects
        self.drive_publisher = self.create_publisher(JointState, '/drive_state', 10)
        self.corner_publisher = self.create_publisher(JointState, '/corner_state', 10)

    def listener_callback(self, msg):
        drive_msg = JointState()
        drive_msg.header = msg.header
        
        corner_msg = JointState()
        corner_msg.header = msg.header

        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            mapped_name = self.name_mapping.get(name, name)
            
            # Route to correct message based on mapped name
            if 'corner' in mapped_name:
                corner_msg.name.append(mapped_name)
                corner_msg.position.append(pos)
                corner_msg.velocity.append(vel)
                corner_msg.effort.append(eff)
            elif 'drive' in mapped_name:
                drive_msg.name.append(mapped_name)
                drive_msg.position.append(pos)
                drive_msg.velocity.append(vel)
                drive_msg.effort.append(eff)

        self.drive_publisher.publish(drive_msg)
        self.corner_publisher.publish(corner_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()