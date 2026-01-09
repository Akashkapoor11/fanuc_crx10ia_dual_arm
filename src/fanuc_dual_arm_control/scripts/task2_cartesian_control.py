#!/usr/bin/env python3
"""
Task 2: Dual-Arm Cartesian Control
Implements synchronized Cartesian path following with collision avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np


class CartesianControlNode(Node):
    """
    Dual-arm Cartesian control node for Task 2.
    Implements synchronized motion with collision avoidance.
    """

    def __init__(self):
        super().__init__('cartesian_control_node')
        
        # Publishers
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
        
        self.get_logger().info('Cartesian Control Node Started')
        
        # Parameters for collision avoidance
        self.min_distance_between_arms = 0.3  # meters
        
        # Timer for demo execution
        self.timer = self.create_timer(3.0, self.execute_cartesian_demo)
        self.demo_executed = False

    def simple_ik(self, target_x, target_y, target_z, arm='left'):
        """
        Simplified inverse kinematics for demonstration.
        In a real implementation, this would use proper IK solver.
        
        Args:
            target_x, target_y, target_z: Target Cartesian position
            arm: 'left' or 'right'
        
        Returns:
            List of joint angles
        """
        # This is a simplified IK solution for demonstration
        # In production, you would use MoveIt's IK solver or a proper IK library
        
        # Simple geometric solution (placeholder)
        j1 = math.atan2(target_y, target_x)
        r = math.sqrt(target_x**2 + target_y**2)
        j2 = math.atan2(target_z - 0.245, r)
        j3 = 0.5
        j4 = 0.0
        j5 = -(j2 + j3)
        j6 = 0.0
        
        return [j1, j2, j3, j4, j5, j6]

    def generate_circle_path(self, center, radius, num_points=20):
        """
        Generate Cartesian waypoints for a circular path.
        
        Args:
            center: (x, y, z) center of circle
            radius: Circle radius
            num_points: Number of waypoints
        
        Returns:
            List of (x, y, z) waypoints
        """
        waypoints = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = center[2]
            waypoints.append((x, y, z))
        return waypoints

    def check_collision(self, left_pos, right_pos):
        """
        Simple collision checking between arms.
        
        Args:
            left_pos: (x, y, z) position of left end-effector
            right_pos: (x, y, z) position of right end-effector
        
        Returns:
            True if collision detected, False otherwise
        """
        distance = math.sqrt(
            (left_pos[0] - right_pos[0])**2 +
            (left_pos[1] - right_pos[1])**2 +
            (left_pos[2] - right_pos[2])**2
        )
        return distance < self.min_distance_between_arms

    def create_synchronized_trajectory(self, left_waypoints, right_waypoints, duration_per_point=0.5):
        """
        Create synchronized trajectories for both arms.
        
        Args:
            left_waypoints: List of (x, y, z) for left arm
            right_waypoints: List of (x, y, z) for right arm
            duration_per_point: Time between waypoints (seconds)
        
        Returns:
            Tuple of (left_trajectory, right_trajectory)
        """
        left_traj = JointTrajectory()
        left_traj.joint_names = self.left_joints
        
        right_traj = JointTrajectory()
        right_traj.joint_names = self.right_joints
        
        for i, (left_wp, right_wp) in enumerate(zip(left_waypoints, right_waypoints)):
            # Check for collision
            if self.check_collision(left_wp, right_wp):
                self.get_logger().warn(f'Collision detected at waypoint {i}, skipping')
                continue
            
            # Compute IK for both arms
            left_joints = self.simple_ik(left_wp[0], left_wp[1], left_wp[2], 'left')
            right_joints = self.simple_ik(right_wp[0], right_wp[1], right_wp[2], 'right')
            
            # Create trajectory points
            time_from_start = (i + 1) * duration_per_point
            
            left_point = JointTrajectoryPoint()
            left_point.positions = left_joints
            left_point.time_from_start = Duration(
                sec=int(time_from_start),
                nanosec=int((time_from_start % 1) * 1e9)
            )
            left_traj.points.append(left_point)
            
            right_point = JointTrajectoryPoint()
            right_point.positions = right_joints
            right_point.time_from_start = Duration(
                sec=int(time_from_start),
                nanosec=int((time_from_start % 1) * 1e9)
            )
            right_traj.points.append(right_point)
        
        return left_traj, right_traj

    def execute_cartesian_demo(self):
        """Execute Task 2: Synchronized Cartesian control demonstration."""
        if self.demo_executed:
            return
        
        self.get_logger().info('='*50)
        self.get_logger().info('Executing Task 2: Dual-Arm Cartesian Control')
        self.get_logger().info('='*50)
        
        # Define circle parameters for both arms
        left_center = (0.4, 0.3, 0.8)   # Left arm circle center
        right_center = (0.4, -0.3, 0.8)  # Right arm circle center
        radius = 0.1
        
        # Generate circular paths
        left_waypoints = self.generate_circle_path(left_center, radius, num_points=30)
        right_waypoints = self.generate_circle_path(right_center, radius, num_points=30)
        
        self.get_logger().info(f'Generated {len(left_waypoints)} waypoints for each arm')
        self.get_logger().info(f'Left arm circle center: {left_center}')
        self.get_logger().info(f'Right arm circle center: {right_center}')
        self.get_logger().info(f'Circle radius: {radius}m')
        
        # Create synchronized trajectories
        left_traj, right_traj = self.create_synchronized_trajectory(
            left_waypoints,
            right_waypoints,
            duration_per_point=0.3
        )
        
        # Publish trajectories
        self.get_logger().info('Publishing synchronized Cartesian trajectories...')
        self.left_arm_pub.publish(left_traj)
        self.right_arm_pub.publish(right_traj)
        
        self.get_logger().info('Task 2 Demo: Both arms drawing circles simultaneously!')
        self.get_logger().info('Collision avoidance: Active')
        self.get_logger().info(f'Minimum safe distance: {self.min_distance_between_arms}m')
        
        self.demo_executed = True


def main(args=None):
    rclpy.init(args=args)
    node = CartesianControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
