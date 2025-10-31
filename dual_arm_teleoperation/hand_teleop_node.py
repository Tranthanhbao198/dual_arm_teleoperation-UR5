#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import cv2
import mediapipe as mp
import numpy as np
import math

class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # MediaPipe hand tracking
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # OpenCV camera
        self.cap = cv2.VideoCapture(0)  # 0 for default camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
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
        
        # Current positions (home pose)
        self.left_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        self.right_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
        
        # Workspace mapping
        self.workspace_scale = 2.0  # Scale factor for hand movement
        
        # Smoothing
        self.smoothing_factor = 0.3
        
        self.get_logger().info('✋ Hand Teleoperation Node Started!')
        self.get_logger().info('📹 Camera initialized')
        self.get_logger().info('🎮 Controls:')
        self.get_logger().info('   - Left hand controls LEFT arm')
        self.get_logger().info('   - Right hand controls RIGHT arm')
        self.get_logger().info('   - Pinch gesture to close gripper')
        self.get_logger().info('   - Press Q to quit')
        
        # Start camera processing in separate thread
        import threading
        self.camera_thread = threading.Thread(target=self.process_camera, daemon=True)
        self.camera_thread.start()
    
    def calculate_pinch_distance(self, landmarks):
        """Calculate distance between thumb tip and index finger tip"""
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        
        distance = math.sqrt(
            (thumb_tip.x - index_tip.x)**2 + 
            (thumb_tip.y - index_tip.y)**2
        )
        return distance
    
    def map_hand_to_joint(self, hand_landmarks, handedness):
        """Map hand position to robot joint positions"""
        
        # Get wrist position (landmark 0)
        wrist = hand_landmarks.landmark[0]
        
        # Get index finger tip for orientation
        index_tip = hand_landmarks.landmark[8]
        
        # Map hand position to robot workspace
        # X: left-right movement (shoulder pan)
        # Y: up-down movement (shoulder lift + elbow)
        # Rotation: hand orientation (wrist rotation)
        
        # Shoulder pan: map X position (0.0 to 1.0 → -π/2 to π/2)
        shoulder_pan = (wrist.x - 0.5) * math.pi * self.workspace_scale
        shoulder_pan = np.clip(shoulder_pan, -math.pi/2, math.pi/2)
        
        # Shoulder lift & elbow: map Y position
        # Y close to 0 (top) → arm up, Y close to 1 (bottom) → arm down
        y_normalized = 1.0 - wrist.y  # Invert Y (0 at bottom, 1 at top)
        
        # Map to shoulder lift angle
        shoulder_lift = -math.pi/2 - (y_normalized - 0.5) * math.pi
        shoulder_lift = np.clip(shoulder_lift, -math.pi, 0)
        
        # Map to elbow angle (keep arm extended)
        elbow = math.pi/2 + (y_normalized - 0.5) * math.pi
        elbow = np.clip(elbow, 0, math.pi)
        
        # Wrist orientation: calculate angle from wrist to index finger
        dx = index_tip.x - wrist.x
        dy = index_tip.y - wrist.y
        hand_angle = math.atan2(dy, dx)
        
        # Map to wrist joints
        wrist_1 = -shoulder_lift - elbow  # Keep end-effector level
        wrist_2 = 0.0
        wrist_3 = hand_angle * 0.5  # Scale down rotation
        
        # Calculate pinch distance for gripper
        pinch_dist = self.calculate_pinch_distance(hand_landmarks.landmark)
        
        # Map pinch to gripper opening (0.04 = open, 0.0 = closed)
        # Pinch distance < 0.05 = closed
        if pinch_dist < 0.05:
            gripper_opening = 0.0
        else:
            gripper_opening = 0.04
        
        return [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, 
                gripper_opening, -gripper_opening]
    
    def smooth_position(self, current, target, alpha):
        """Smooth position transition"""
        return [c + alpha * (t - c) for c, t in zip(current, target)]
    
    def process_camera(self):
        """Process camera feed and hand tracking"""
        
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to read camera frame')
                continue
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Draw hand landmarks
            if results.multi_hand_landmarks:
                for idx, (hand_landmarks, handedness) in enumerate(
                    zip(results.multi_hand_landmarks, results.multi_handedness)
                ):
                    # Draw landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, 
                        hand_landmarks, 
                        self.mp_hands.HAND_CONNECTIONS
                    )
                    
                    # Get hand label (Left or Right)
                    hand_label = handedness.classification[0].label
                    
                    # Map hand to joint positions
                    target_positions = self.map_hand_to_joint(hand_landmarks, hand_label)
                    
                    # Update corresponding arm with smoothing
                    if hand_label == "Left":
                        # Left hand controls RIGHT arm (mirrored)
                        self.right_positions = self.smooth_position(
                            self.right_positions, 
                            target_positions, 
                            self.smoothing_factor
                        )
                        
                        # Draw label
                        cv2.putText(frame, "RIGHT ARM", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        # Right hand controls LEFT arm (mirrored)
                        self.left_positions = self.smooth_position(
                            self.left_positions, 
                            target_positions, 
                            self.smoothing_factor
                        )
                        
                        # Draw label
                        cv2.putText(frame, "LEFT ARM", (450, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    
                    # Calculate and display pinch distance
                    pinch_dist = self.calculate_pinch_distance(hand_landmarks.landmark)
                    gripper_state = "CLOSED" if pinch_dist < 0.05 else "OPEN"
                    
                    # Display gripper state
                    x_pos = 10 if hand_label == "Left" else 450
                    cv2.putText(frame, f"Gripper: {gripper_state}", (x_pos, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Display instructions
            cv2.putText(frame, "Press Q to quit", (200, 470), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow('Hand Teleoperation', frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quitting...')
                break
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.left_joints + self.right_joints
        msg.position = self.left_positions + self.right_positions
        self.joint_pub.publish(msg)
    
    def cleanup(self):
        """Cleanup resources"""
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = HandTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
