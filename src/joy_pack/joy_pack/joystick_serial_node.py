#!/usr/bin/env python3
"""
Joystick to Motor Command Node

This node:
1. Subscribes to /joy topic
2. Processes buttons (0-11) to select motor (1-6)
3. Processes Y-axis for speed/direction
4. Publishes motor commands to /motor_commands topic

Motor command format: [motor1_pwm, motor2_pwm, ..., motor6_pwm]
PWM range: -255 to +255 (positive = forward, negative = reverse)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
import sys

class JoystickToMotorNode(Node):
    def __init__(self):
        super().__init__('joystick_serial_node')
        
        # Parameters
        self.declare_parameter('deadband', 0.1)
        self.declare_parameter('button_mapping', 'auto')  # 'auto' or 'manual'
        
        self.deadband = self.get_parameter('deadband').value
        
        # Motor PWM values (-255 to +255)
        self.motor_pwm = [0] * 6
        self.selected_motor = -1
        
        # Subscribe to joy
        self.joy_sub = self.create_subscription(
            Joy,
            'joy1',
            self.joy_callback,
            10
        )
        
        # Publish motor commands
        self.motor_pub = self.create_publisher(
            Int16MultiArray,
            'motor_commands',
            10
        )
        
        # Timer to publish at 20Hz
        self.timer = self.create_timer(0.05, self.publish_motor_commands)
        
        # Track last values to detect changes
        self.last_motor_pwm = [0] * 6
        self.last_selected = -1
        
        self.get_logger().info('🎮 Joystick to Motor Node Started')
        self.get_logger().info('📡 Subscribing to: /joy')
        self.get_logger().info('📤 Publishing to: /motor_commands')
        self.get_logger().info('⚙️  Deadband: {}'.format(self.deadband))
        self.get_logger().info('🔘 Press buttons 0-11 to control motors 1-6')
        self.get_logger().info('🕹️  Y-axis (axis 1) controls speed/direction')
        
    def joy_callback(self, msg):
        """Process joystick data"""
        
        # Get Y-axis value (axis 1 = forward/back)
        if len(msg.axes) > 1:
            y_axis = msg.axes[1]
        else:
            y_axis = 0.0
            
        # Apply deadband
        if abs(y_axis) < self.deadband:
            y_axis = 0.0
            
        # Convert to PWM (-255 to +255)
        # Positive y_axis = forward, negative = reverse
        pwm_value = int(y_axis * 255)
        pwm_value = max(-255, min(255, pwm_value))
        
        # Determine direction
        is_forward = (y_axis > 0)
        
        # Check which button is pressed
        # Try buttons 0-5 first
        self.selected_motor = -1
        for i in range(6):
            if len(msg.buttons) > i and msg.buttons[i]:
                self.selected_motor = i
                break
        
        # If not found, try buttons 6-11
        if self.selected_motor == -1:
            for i in range(6):
                button_idx = i + 6
                if len(msg.buttons) > button_idx and msg.buttons[button_idx]:
                    self.selected_motor = i
                    break
        
        # Update motor PWM values
        if self.selected_motor >= 0:
            # Reset all to 0
            self.motor_pwm = [0] * 6
            # Set selected motor
            self.motor_pwm[self.selected_motor] = pwm_value
            
            # Log if changed
            if (self.selected_motor != self.last_selected or 
                self.motor_pwm[self.selected_motor] != self.last_motor_pwm[self.selected_motor]):
                
                direction = "FWD" if is_forward else "REV" if y_axis < 0 else "STOP"
                self.get_logger().info(
                    f'🎯 Motor {self.selected_motor + 1}: {direction} {abs(pwm_value)}/255'
                )
                self.last_selected = self.selected_motor
                self.last_motor_pwm = self.motor_pwm.copy()
        else:
            # No button pressed - stop all
            self.motor_pwm = [0] * 6
            if self.last_selected != -1:
                self.get_logger().info('⏸️  All motors stopped')
                self.last_selected = -1
                self.last_motor_pwm = [0] * 6
    
    def publish_motor_commands(self):
        """Publish motor commands"""
        msg = Int16MultiArray()
        msg.data = self.motor_pwm
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToMotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Stop all motors on shutdown
        msg = Int16MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0]
        node.motor_pub.publish(msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
