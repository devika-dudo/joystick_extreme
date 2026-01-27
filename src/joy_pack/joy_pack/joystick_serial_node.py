#!/usr/bin/env python3
"""
Button-Only Motor Control Node with Individual PWM Control

This node:
1. Subscribes to /joy topic
2. Uses 12 buttons to control 6 motors (2 buttons per motor)
3. Each motor can have its own PWM speed
4. Publishes motor commands to /motor_commands topic

Button Mapping:
- Button 0:  Motor 1 Forward
- Button 1:  Motor 1 Backward
- Button 2:  Motor 2 Forward
- Button 3:  Motor 2 Backward
- Button 4:  Motor 3 Forward
- Button 5:  Motor 3 Backward
- Button 6:  Motor 4 Forward
- Button 7:  Motor 4 Backward
- Button 8:  Motor 5 Forward
- Button 9:  Motor 5 Backward
- Button 10: Motor 6 Forward
- Button 11: Motor 6 Backward

Motor command format: [motor1_pwm, motor2_pwm, ..., motor6_pwm]
PWM range: -255 to +255 (positive = forward, negative = reverse)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

class ButtonMotorControlNode(Node):
    def __init__(self):
        super().__init__('button_motor_control_node')
        
        # Parameters - Individual PWM for each motor
        self.declare_parameter('motor_1_speed', 200)
        self.declare_parameter('motor_2_speed', 200)
        self.declare_parameter('motor_3_speed', 200)
        self.declare_parameter('motor_4_speed', 100)
        self.declare_parameter('motor_5_speed', 200)
        self.declare_parameter('motor_6_speed', 200)
        
        # Get individual motor speeds
        self.motor_speeds = [
            self.get_parameter('motor_1_speed').value,
            self.get_parameter('motor_2_speed').value,
            self.get_parameter('motor_3_speed').value,
            self.get_parameter('motor_4_speed').value,
            self.get_parameter('motor_5_speed').value,
            self.get_parameter('motor_6_speed').value
        ]
        
        # Motor PWM values (-255 to +255)
        self.motor_pwm = [0] * 6
        
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
        
        self.get_logger().info('🎮 Button Motor Control Node Started')
        self.get_logger().info('📡 Subscribing to: /joy1')
        self.get_logger().info('📤 Publishing to: /motor_commands')
        self.get_logger().info('')
        self.get_logger().info('⚙️  Individual Motor Speeds:')
        for i, speed in enumerate(self.motor_speeds):
            self.get_logger().info(f'   Motor {i+1}: {speed} PWM')
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════╗')
        self.get_logger().info('║        BUTTON MAPPING (12 buttons)        ║')
        self.get_logger().info('╠═══════════════════════════════════════════╣')
        self.get_logger().info('║ Button 0:  Motor 1 FORWARD                ║')
        self.get_logger().info('║ Button 1:  Motor 1 BACKWARD               ║')
        self.get_logger().info('║ Button 2:  Motor 2 FORWARD                ║')
        self.get_logger().info('║ Button 3:  Motor 2 BACKWARD               ║')
        self.get_logger().info('║ Button 4:  Motor 3 FORWARD                ║')
        self.get_logger().info('║ Button 5:  Motor 3 BACKWARD               ║')
        self.get_logger().info('║ Button 6:  Motor 4 FORWARD                ║')
        self.get_logger().info('║ Button 7:  Motor 4 BACKWARD               ║')
        self.get_logger().info('║ Button 8:  Motor 5 FORWARD                ║')
        self.get_logger().info('║ Button 9:  Motor 5 BACKWARD               ║')
        self.get_logger().info('║ Button 10: Motor 6 FORWARD                ║')
        self.get_logger().info('║ Button 11: Motor 6 BACKWARD               ║')
        self.get_logger().info('╚═══════════════════════════════════════════╝')
        self.get_logger().info('')
        
    def joy_callback(self, msg):
        """Process joystick button data"""
        
        # Reset all motors
        self.motor_pwm = [0] * 6
        
        # Check each button (need at least 12 buttons)
        if len(msg.buttons) < 12:
            self.get_logger().warn(
                f'⚠️  Only {len(msg.buttons)} buttons available, need 12!',
                throttle_duration_sec=2.0
            )
            return
        
        # Process each motor (0-5)
        for motor_idx in range(6):
            fwd_button = motor_idx * 2      # 0, 2, 4, 6, 8, 10
            bwd_button = motor_idx * 2 + 1  # 1, 3, 5, 7, 9, 11
            
            # Check forward button - use individual motor speed
            if msg.buttons[fwd_button]:
                self.motor_pwm[motor_idx] = self.motor_speeds[motor_idx]
                
            # Check backward button - use individual motor speed (negative)
            elif msg.buttons[bwd_button]:
                self.motor_pwm[motor_idx] = -self.motor_speeds[motor_idx]
        
        # Log changes
        if self.motor_pwm != self.last_motor_pwm:
            active_motors = []
            for i, pwm in enumerate(self.motor_pwm):
                if pwm != 0:
                    direction = "FWD" if pwm > 0 else "REV"
                    active_motors.append(f"M{i+1}:{direction}({abs(pwm)})")
            
            if active_motors:
                self.get_logger().info(f'🎯 {" | ".join(active_motors)}')
            else:
                self.get_logger().info('⏸️  All motors stopped')
            
            self.last_motor_pwm = self.motor_pwm.copy()
    
    def publish_motor_commands(self):
        """Publish motor commands"""
        msg = Int16MultiArray()
        msg.data = self.motor_pwm
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ButtonMotorControlNode()
    
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
