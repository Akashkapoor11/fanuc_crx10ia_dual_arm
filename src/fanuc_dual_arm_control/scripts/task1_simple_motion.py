#!/usr/bin/env python3
"""
Task 1: Simple Motion Planning Demo
Demonstrates moving each arm to specific poses independently
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class SimpleMotionDemo(Node):
    """
    Simple motion planning demo for Task 1.
    Moves left and right arms to predefined poses.
    """

    def __init__(self):
        super().__init__('simple_motion_demo')
        
        # Publishers for joint trajectories
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        
        # Joint names
        self.left_joints = [
            'left_joint_1', 'left_joint_2', 'left_joint_3',
            'left_joint_4', 'left_joint_5', 'left_joint_6'
        ]
        self.right_joints = [
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        
        self.get_logger().info('Simple Motion Demo Node Started')
        
        # Wait a bit then execute demo
        self.timer = self.create_timer(2.0, self.execute_demo)
        self.demo_executed = False

    def create_trajectory(self, joint_names, positions, duration_sec=3.0):
        """
        Create a joint trajectory message.
        
        Args:
            joint_names: List of joint names
            positions: List of target joint positions (radians)
            duration_sec: Time to reach target (seconds)
        
        Returns:
            JointTrajectory message
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [point]
        return trajectory

    def execute_demo(self):
        """Execute the simple motion demonstration."""
        if self.demo_executed:
            return
        
        self.get_logger().info('Executing Task 1: Simple Motion Planning Demo')
        
        # Define target poses for left arm (reaching forward and up)
        left_target = [
            0.0,      # joint_1: 0 degrees
            -0.5,     # joint_2: -30 degrees
            1.0,      # joint_3: 60 degrees
            0.0,      # joint_4: 0 degrees
            0.5,      # joint_5: 30 degrees
            0.0       # joint_6: 0 degrees
        ]
        
        # Define target poses for right arm (reaching forward and up, mirrored)
        right_target = [
            0.0,      # joint_1: 0 degrees
            -0.5,     # joint_2: -30 degrees
            1.0,      # joint_3: 60 degrees
            0.0,      # joint_4: 0 degrees
            0.5,      # joint_5: 30 degrees
            0.0       # joint_6: 0 degrees
        ]
        
        # Create and publish trajectories
        left_traj = self.create_trajectory(self.left_joints, left_target, duration_sec=4.0)
        right_traj = self.create_trajectory(self.right_joints, right_target, duration_sec=4.0)
        
        self.get_logger().info('Moving left arm to target pose...')
        self.left_arm_pub.publish(left_traj)
        
        self.get_logger().info('Moving right arm to target pose...')
        self.right_arm_pub.publish(right_traj)
        
        self.get_logger().info('Task 1 Demo: Both arms moving to target poses!')
        self.get_logger().info('This demonstrates simple motion planning with MoveIt 2')
        
        self.demo_executed = True


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotionDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
