#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        # Joint names
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
        
        # Current positions (upright pose)
        self.left_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        self.right_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        
        # Control parameters
        self.active_arm = 'left'  # 'left' or 'right'
        self.active_joint = 0  # 0-5 for arm joints
        self.step_size = 0.1  # radians
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
        self.get_logger().info('Keyboard Teleop Node Started!')
        
        # Start keyboard input thread
        import threading
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()
        
    def print_instructions(self):
        print("\n" + "="*60)
        print("🤖 DUAL ARM KEYBOARD TELEOPERATION")
        print("="*60)
        print("\n🎮 CONTROLS:")
        print("  TAB       : Switch arm (Left ↔ Right)")
        print("  1-6       : Select joint (1=shoulder_pan ... 6=wrist_3)")
        print("  +/-       : Increase/Decrease selected joint")
        print("  w/s       : Increase/Decrease joint (alternative)")
        print("  o/c       : Open/Close gripper")
        print("  h         : Home position (upright)")
        print("  r         : Reset to zero")
        print("  q         : Quit")
        print("="*60)
        print(f"Active Arm: LEFT | Active Joint: 1 (Shoulder Pan)")
        print("="*60 + "\n")
    
    def keyboard_loop(self):
        """Main keyboard input loop"""
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    tty.setraw(sys.stdin.fileno())
                    key = sys.stdin.read(1)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                    self.process_key(key)
            except:
                break
    
    def process_key(self, key):
        """Process keyboard input"""
        
        # Switch arm
        if key == '\t':  # TAB
            self.active_arm = 'right' if self.active_arm == 'left' else 'left'
            print(f"\n✓ Switched to {self.active_arm.upper()} arm")
        
        # Select joint (1-6)
        elif key in ['1', '2', '3', '4', '5', '6']:
            self.active_joint = int(key) - 1
            joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 
                          'Wrist 1', 'Wrist 2', 'Wrist 3']
            print(f"\n✓ Selected: {joint_names[self.active_joint]}")
        
        # Increase joint angle
        elif key in ['+', '=', 'w', 'W']:
            self.change_joint(self.step_size)
        
        # Decrease joint angle
        elif key in ['-', '_', 's', 'S']:
            self.change_joint(-self.step_size)
        
        # Open gripper
        elif key in ['o', 'O']:
            self.set_gripper(0.04)
            print(f"\n✓ {self.active_arm.upper()} gripper OPENED")
        
        # Close gripper
        elif key in ['c', 'C']:
            self.set_gripper(0.0)
            print(f"\n✓ {self.active_arm.upper()} gripper CLOSED")
        
        # Home position
        elif key in ['h', 'H']:
            self.home_position()
            print("\n✓ Moving to HOME position")
        
        # Reset to zero
        elif key in ['r', 'R']:
            self.reset_position()
            print("\n✓ RESET to zero position")
        
        # Quit
        elif key in ['q', 'Q']:
            print("\n\n👋 Goodbye!")
            raise KeyboardInterrupt
    
    def change_joint(self, delta):
        """Change active joint angle"""
        if self.active_arm == 'left':
            self.left_positions[self.active_joint] += delta
            self.left_positions[self.active_joint] = max(-3.14, min(3.14, 
                self.left_positions[self.active_joint]))
            current = self.left_positions[self.active_joint]
        else:
            self.right_positions[self.active_joint] += delta
            self.right_positions[self.active_joint] = max(-3.14, min(3.14, 
                self.right_positions[self.active_joint]))
            current = self.right_positions[self.active_joint]
        
        joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 
                      'Wrist 1', 'Wrist 2', 'Wrist 3']
        print(f"\r{self.active_arm.upper()} {joint_names[self.active_joint]}: {current:.2f} rad", 
              end='', flush=True)
    
    def set_gripper(self, opening):
        """Set gripper opening"""
        if self.active_arm == 'left':
            self.left_positions[6] = opening
            self.left_positions[7] = -opening
        else:
            self.right_positions[6] = opening
            self.right_positions[7] = -opening
    
    def home_position(self):
        """Move to home (upright) position"""
        home = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]
        if self.active_arm == 'left':
            self.left_positions[0:6] = home
        else:
            self.right_positions[0:6] = home
    
    def reset_position(self):
        """Reset to zero position"""
        if self.active_arm == 'left':
            self.left_positions = [0.0] * 8
        else:
            self.right_positions = [0.0] * 8
    
    def publish_joint_states(self):
        """Publish joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.left_joints + self.right_joints
        msg.position = self.left_positions + self.right_positions
        self.joint_pub.publish(msg)
    
    def cleanup(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
