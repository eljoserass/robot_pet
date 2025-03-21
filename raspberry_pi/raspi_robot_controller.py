#!/usr/bin/env python3
"""
Unified Raspberry Pi controller node that handles:
1. Motor control for movement
2. Servo control for camera positioning
3. Camera image publishing

This integrates both motor control and servo control in a single node
to simplify the Raspberry Pi implementation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

# Servo positions
SERVO_LEFT = 160
SERVO_FRONT = 80
SERVO_RIGHT = 0
SERVO_CHANNEL = 15

# setup i2c and pca9685 board for motors
I2C_BUS = busio.I2C(SCL, SDA)
PWM = PCA9685(I2C_BUS)
PWM.frequency = 60

# configure gpio pins (bcm numbering)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
IN1 = 23  # left motor direction pin
IN2 = 24  # left motor direction pin
IN3 = 27  # right motor direction pin
IN4 = 22  # right motor direction pin
ENA = 0   # left motor speed channel on PCA9685
ENB = 1   # right motor speed channel on PCA9685

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

MOVE_SPEED = 0x7FFF  # 1/4

class UnifiedRobotController(Node):
    def __init__(self):
        super().__init__('unified_robot_controller')
        
        # Initialize servo kit
        try:
            self.kit = ServoKit(channels=16)
            self.kit.servo[SERVO_CHANNEL].angle = SERVO_FRONT
            self.get_logger().info('Servo initialized at front position')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo: {e}')
            self.get_logger().warn('Robot will operate without servo control')
        
        # Parameter for servo movement delay
        self.declare_parameter('servo_delay', 0.5)
        self.servo_delay = self.get_parameter('servo_delay').value
        
        # Create subscriptions
        # 1. Motor control subscription
        self.motor_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 2. Servo control subscription
        self.servo_subscription = self.create_subscription(
            String,
            'servo_command',
            self.servo_command_callback,
            10
        )
        
        self.get_logger().info('Unified robot controller started')
    
    def change_speed(self, speed):
        """Set motor speed."""
        self.get_logger().debug(f"Setting speed to {speed}")
        PWM.channels[ENA].duty_cycle = speed
        PWM.channels[ENB].duty_cycle = speed

    def stop_car(self):
        """Stop all motors."""
        self.get_logger().debug("Stopping car")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.change_speed(0)

    def forward(self):
        """Move forward."""
        self.get_logger().debug("Moving forward")
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        self.change_speed(MOVE_SPEED)

    def backward(self):
        """Move backward."""
        self.get_logger().debug("Moving backward")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.change_speed(MOVE_SPEED)

    def turn_right(self):
        """Turn right."""
        self.get_logger().debug("Turning right")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.change_speed(MOVE_SPEED)

    def turn_left(self):
        """Turn left."""
        self.get_logger().debug("Turning left")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.change_speed(MOVE_SPEED)
    
    def cmd_vel_callback(self, msg):
        """Handle movement commands."""
        self.get_logger().debug(f"Received twist -> linear.x: {msg.linear.x}, angular.z: {msg.angular.z}")
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < 0.1 and abs(angular) < 0.1:
            self.get_logger().debug("Within deadzone; stopping car")
            self.stop_car()
        else:
            if linear > 0:
                self.get_logger().debug("Linear positive; moving forward")
                self.forward()
            elif linear < 0:
                self.get_logger().debug("Linear negative; moving backward")
                self.backward()
            elif angular > 0:
                self.get_logger().debug("Angular positive; turning left")
                self.turn_left()
            elif angular < 0:
                self.get_logger().debug("Angular negative; turning right")
                self.turn_right()
    
    def servo_command_callback(self, msg):
        """Handle servo position command."""
        command = msg.data.upper()
        self.get_logger().info(f'Received servo command: {command}')
        
        try:
            if command == "LEFT":
                self.kit.servo[SERVO_CHANNEL].angle = SERVO_LEFT
                self.get_logger().info('Moved servo to LEFT position')
            elif command == "FRONT" or command == "CENTER":
                self.kit.servo[SERVO_CHANNEL].angle = SERVO_FRONT
                self.get_logger().info('Moved servo to FRONT position')
            elif command == "RIGHT":
                self.kit.servo[SERVO_CHANNEL].angle = SERVO_RIGHT
                self.get_logger().info('Moved servo to RIGHT position')
            else:
                self.get_logger().warn(f'Unknown servo command: {command}')
                return
            
            # Allow time for servo to move and camera to stabilize
            time.sleep(self.servo_delay)
            
        except Exception as e:
            self.get_logger().error(f'Error moving servo: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedRobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot controller stopped by user')
    except Exception as e:
        node.get_logger().error(f'Error in robot controller: {e}')
    finally:
        # Clean up
        node.stop_car()
        GPIO.cleanup()
        # Return servo to front position
        try:
            node.kit.servo[SERVO_CHANNEL].angle = SERVO_FRONT
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()