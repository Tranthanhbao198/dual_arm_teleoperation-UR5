#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class DualArmController(Node):
    def __init__(self):
        super().__init__('dual_arm_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # Joint names for left and right arms
        self.left_joints = [
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
            'left_gripper_joint1',
            'left_gripper_joint2'
        ]
        
        self.right_joints = [
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint',
            'right_gripper_joint1',
            'right_gripper_joint2'
        ]
        
        # Initial positions (upright pose)
        self.left_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        self.right_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        
        # Animation parameters
        self.time = 0.0
        self.animation_enabled = True
        
        self.get_logger().info('Dual Arm Controller started!')
        self.get_logger().info('Publishing joint states at 50Hz')
        
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Combine joint names
        msg.name = self.left_joints + self.right_joints
        
        # Simple wave animation (optional)
        if self.animation_enabled:
            self.time += 0.02
            
            # Left arm waves
            self.left_positions[0] = 0.3 * math.sin(self.time)  # shoulder pan
            self.left_positions[4] = 0.5 * math.sin(self.time * 1.5)  # wrist 2
            
            # Right arm waves (opposite phase)
            self.right_positions[0] = 0.3 * math.sin(self.time + math.pi)
            self.right_positions[4] = 0.5 * math.sin(self.time * 1.5 + math.pi)
        
        # Combine positions
        msg.position = self.left_positions + self.right_positions
        
        # Publish
        self.joint_pub.publish(msg)

    def set_left_arm_position(self, positions):
        """Set left arm joint positions"""
        if len(positions) == 6:
            self.left_positions[0:6] = positions
            self.get_logger().info(f'Left arm position updated: {positions}')
    
    def set_right_arm_position(self, positions):
        """Set right arm joint positions"""
        if len(positions) == 6:
            self.right_positions[0:6] = positions
            self.get_logger().info(f'Right arm position updated: {positions}')
    
    def set_gripper(self, arm, opening):
        """
        Set gripper opening
        arm: 'left' or 'right'
        opening: 0.0 (closed) to 0.04 (open)
        """
        opening = max(0.0, min(0.04, opening))
        
        if arm == 'left':
            self.left_positions[6] = opening
            self.left_positions[7] = -opening
        elif arm == 'right':
            self.right_positions[6] = opening
            self.right_positions[7] = -opening
        
        self.get_logger().info(f'{arm.capitalize()} gripper set to {opening:.3f}')
    
    def stop_animation(self):
        """Stop automatic animation"""
        self.animation_enabled = False
        self.get_logger().info('Animation stopped')
    
    def start_animation(self):
        """Start automatic animation"""
        self.animation_enabled = True
        self.get_logger().info('Animation started')


def main(args=None):
    rclpy.init(args=args)
    
    controller = DualArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Dual Arm Controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
