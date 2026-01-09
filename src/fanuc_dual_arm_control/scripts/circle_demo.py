#!/usr/bin/env python3
"""
Circle Drawing Demo
Demonstrates synchronized circular motion with both arms
"""

import rclpy
from task2_cartesian_control import CartesianControlNode


def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianControlNode()
    node.get_logger().info('Circle Drawing Demo Started')
    node.get_logger().info('Both arms will draw circles simultaneously')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
