#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReader(Node):
    def __init__(self):
        super().__init__('joint_state_reader')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # Queue size
        )
        self.get_logger().info('JointStateReader Node Initialized.')

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Joint Names: {msg.name}')
        self.get_logger().info(f'Joint Positions: {msg.position}')
        self.get_logger().info(f'Joint Velocities: {msg.velocity}')
        self.get_logger().info(f'Joint Efforts: {msg.effort}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

