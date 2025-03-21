#!/usr/bin/env python3
print("IMPROVED MULTI-VIEW NODE WITH DEBUG STARTED")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from threading import Lock
import base64
import os
import json
import socket
from openai import OpenAI

from google import genai
from google.genai import types
from pydantic import BaseModel
from enum import Enum

class MovementOptions(str, Enum):
    forward = "FORWARD"
    backward = "BACKWARD"
    left = "LEFT"
    right = "RIGHT"
    stop = "STOP"

class Movement(BaseModel):
    reasoning: str
    direction: MovementOptions

class DebugNavigationNode(Node):
    def __init__(self):
        super().__init__('debug_navigation_node')

        # Parameters
        self.declare_parameter('target_location', 'go to the kitchen')
        self.declare_parameter('model_name', 'gpt-4o')
        self.declare_parameter('api_key', '')  # Should be set via ros args or environment variable
        self.declare_parameter('servo_delay', 0.5)  # Time to wait for servo to settle
        self.declare_parameter('min_cmd_duration', 0.5)  # Minimum time to send command
        self.declare_parameter('continuous_movement', True)  # Keep moving instead of stop-move-stop
        self.declare_parameter('enable_debug', True)  # Enable debug mode
        self.declare_parameter('debug_port', 9876)  # Port for communication with debug tools

        # Initialize variables
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = Lock()
        self.target_location = self.get_parameter('target_location').value
        self.is_arrived = False
        self.model_name = self.get_parameter('model_name').value
        self.servo_delay = self.get_parameter('servo_delay').value
        self.min_cmd_duration = self.get_parameter('min_cmd_duration').value
        self.continuous_movement = self.get_parameter('continuous_movement').value
        self.enable_debug = self.get_parameter('enable_debug').value
        self.debug_port = self.get_parameter('debug_port').value

        # Debug socket for conversation tracking
        self.debug_socket = None
        if self.enable_debug:
            try:
                self.debug_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.debug_socket.connect(('localhost', self.debug_port))
                self.get_logger().info(f"Connected to conversation tracker on port {self.debug_port}")
            except Exception as e:
                self.get_logger().warn(f"Could not connect to conversation tracker: {e}")
                self.get_logger().info("Start conversation_tracker.py in another terminal to see the conversation")
                self.debug_socket = None

        # Multi-view image storage
        self.left_image = None
        self.front_image = None
        self.right_image = None

        # New state tracking
        self.currently_moving = False
        self.current_direction = None
        self.movement_start_time = None
        self.last_scan_time = 0
        self.scan_interval = 3.0  # Scan every 3 seconds while moving
        self.scan_in_progress = False
        self.scan_state = 0

        self.system_prompt = f"""You are a home navigation robot.
                    You control a little two-wheeled differential robot with an RGB camera on the front.
                    You have access to THREE camera views: LEFT, FRONT, and RIGHT views simultaneously.
                    The three views give you wider situational awareness of your surroundings.

                    You can move around with the following commands:
                        - FORWARD: Move one step forward.
                        - BACKWARD: Move one step back, useful if you get stuck.
                        - LEFT: Turn slightly to the left.
                        - RIGHT: Turn slightly to the right.
                    You can use any combination of these commands.
                    After each command you will see the results from all three camera positions.
                    You will follow your owner's queries to navigate to the requested location.
                    When you consider you've reached said location say STOP.

                    Your owner sent you the following query: {self.target_location}

                    If you see that after a few execution of the same command the images don't change, it means you are stuck.
                    Beware that the environment is very similar so be careful to not be confused, we can say that if after 5 images everything looks the same, you are stuck

                    RESPOND TO THE USER ONLY WITH THE COMMAND SELECTED
                    """

        # Get API key from parameter or environment variable
        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('OPENAI_API_KEY')
            if not api_key:
                self.get_logger().error('No OpenAI API key provided! Set it via ROS parameter or OPENAI_API_KEY environment variable')

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key=api_key, base_url="https://generativelanguage.googleapis.com/v1beta/openai/")

        # Initialize Gemini client
        self.gemini_client = genai.Client(api_key=os.environ.get('GEMINI_API_KEY'))
        self.gemini_model = "gemini-2.0-flash"  # Use an appropriate Gemini model
        self.gemini_config = types.GenerateContentConfig(
            response_mime_type='application/json',
            response_schema=Movement
        )

        # Initialize message history
        self.messages = [{"role": "system", "content": self.system_prompt}]
        self.send_debug_message(self.messages[0])  # Send system prompt to debug tracker

        # Message history management
        self.max_message_history = 10  # Only keep the last N interactions

        # Create subscribers and publishers
        self.create_subscription(
            Image,
            '/image',  # Camera topic
            self.image_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Add publisher for servo control
        self.servo_pub = self.create_publisher(
            String,
            'servo_command',
            10)

        # Timer for main loop - run at 10Hz for more responsive control
        self.timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info(f'Debug navigation node started. Target: {self.target_location}')

        # Start by setting servo to front position
        self.send_servo_command("FRONT")
        time.sleep(1.0)  # Give time for the servo to settle initially

    def send_debug_message(self, message):
        """Send a message to the conversation tracker."""
        if not self.debug_socket:
            return

        try:
            # Convert the message to JSON and send it
            msg_json = json.dumps(message) + '\n'
            self.debug_socket.sendall(msg_json.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending debug message: {e}")
            # Try to reconnect
            try:
                self.debug_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.debug_socket.connect(('localhost', self.debug_port))
                self.get_logger().info("Reconnected to conversation tracker")
                # Try sending again
                msg_json = json.dumps(message) + '\n'
                self.debug_socket.sendall(msg_json.encode('utf-8'))
            except:
                self.debug_socket = None

    def image_callback(self, msg):
        with self.frame_lock:
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')

    def send_servo_command(self, position):
        """Send command to move servo to the specified position."""
        try:
            msg = String()
            msg.data = position
            self.servo_pub.publish(msg)
            self.get_logger().info(f'Sent servo command: {position}')
            # Allow time for servo movement and image capture
            time.sleep(self.servo_delay)
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending servo command: {e}')
            return False

    def send_movement_command(self, command):
        """Send a movement command and track its execution."""
        twist = Twist()

        # Lower speed values for more gentle turns
        linear_speed = 0.2  # m/s
        angular_speed = 0.15  # rad/s

        # Parse command and set velocities
        if "FORWARD" in command:
            twist.linear.x = linear_speed
        elif "BACKWARD" in command:
            twist.linear.x = -linear_speed
        elif "LEFT" in command:
            twist.angular.z = angular_speed
        elif "RIGHT" in command:
            twist.angular.z = -angular_speed
        elif "STOP" in command:
            # All values are 0 by default
            pass

        # Send the command
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Published movement: {command}')

        # Update movement tracking
        self.currently_moving = (command != "STOP")
        self.current_direction = command
        self.movement_start_time = time.time()

    def query_gemini(self, left_image, front_image, right_image):
        """Query the Gemini model with all three camera views."""
        try:
            from PIL import Image as PILImage
            import io

            # Convert all OpenCV images to PIL format
            left_pil = PILImage.fromarray(cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB))
            front_pil = PILImage.fromarray(cv2.cvtColor(front_image, cv2.COLOR_BGR2RGB))
            right_pil = PILImage.fromarray(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB))

            # Convert to bytes
            left_bytes = io.BytesIO()
            front_bytes = io.BytesIO()
            right_bytes = io.BytesIO()

            left_pil.save(left_bytes, format='JPEG')
            front_pil.save(front_bytes, format='JPEG')
            right_pil.save(right_bytes, format='JPEG')

            left_image_bytes = left_bytes.getvalue()
            front_image_bytes = front_bytes.getvalue()
            right_image_bytes = right_bytes.getvalue()

            # Create message with all images
            content = types.Content(
                role="user",
                parts=[
                    types.Part.from_text("LEFT VIEW:"),
                    types.Part.from_bytes(left_image_bytes, mime_type="image/jpeg"),
                    types.Part.from_text("FRONT VIEW:"),
                    types.Part.from_bytes(front_image_bytes, mime_type="image/jpeg"),
                    types.Part.from_text("RIGHT VIEW:"),
                    types.Part.from_bytes(right_image_bytes, mime_type="image/jpeg")
                ]
            )

            # Manage message history
            if len(self.messages) > self.max_message_history:
                # Keep system prompt and last N-1 exchanges
                self.messages = [self.messages[0]] + self.messages[-(self.max_message_history-1):]

            # Generate response
            response = self.gemini_client.generate_content(
                model=self.gemini_model,
                contents=[content],
                generation_config=self.gemini_config
            )

            # Log and process response
            self.get_logger().info(f"Gemini response: {response.text}")

            # Extract movement direction
            robot_control = response.candidates[0].content.parts[0].parsed
            return robot_control.direction

        except Exception as e:
            self.get_logger().error(f'Error querying Gemini: {e}')
            return "STOP"  # Safe default

    def query_openai(self, left_image, front_image, right_image):
        """Send all three views to OpenAI and get movement instructions."""
        try:
            self.get_logger().info("Encoding frames for OpenAI")
            # Convert all images to base64 for transmission
            _, left_encoded = cv2.imencode('.jpg', left_image)
            _, front_encoded = cv2.imencode('.jpg', front_image)
            _, right_encoded = cv2.imencode('.jpg', right_image)

            left_base64 = base64.b64encode(left_encoded).decode('utf-8')
            front_base64 = base64.b64encode(front_encoded).decode('utf-8')
            right_base64 = base64.b64encode(right_encoded).decode('utf-8')

            self.get_logger().info("Frames encoded")

            # Add previous message content if we have any (for conversation history)
            current_messages = list(self.messages)  # Make a copy

            # Create new message with the three views
            new_message = {
                "role": "user",
                "content": [
                    {"type": "text", "text": "LEFT VIEW:"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{left_base64}",
                        },
                    },
                    {"type": "text", "text": "FRONT VIEW:"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{front_base64}",
                        },
                    },
                    {"type": "text", "text": "RIGHT VIEW:"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{right_base64}",
                        },
                    },
                ],
            }

            # Send this message to debug tracker
            self.send_debug_message(new_message)

            # Add it to current messages for the API call
            current_messages.append(new_message)

            # Get response from OpenAI
            self.get_logger().info(f"Calling {self.model_name}")
            response = self.openai_client.chat.completions.create(
                model=self.gemini_model,
                messages=current_messages,
                max_tokens=50
            )
            self.get_logger().info("Call finished")

            # Extract command from response
            command = response.choices[0].message.content.strip()

            # Create assistant message
            assistant_message = {"role": "assistant", "content": command}

            # Send assistant response to debug tracker
            self.send_debug_message(assistant_message)

            # Add the conversation to our history
            self.messages.append(new_message)
            self.messages.append(assistant_message)

            # Manage message history size
            if len(self.messages) > self.max_message_history + 1:  # +1 for system message
                # Keep system prompt and last N messages
                self.messages = [self.messages[0]] + self.messages[-(self.max_message_history):]

            return command

        except Exception as e:
            self.get_logger().error(f'Error communicating with OpenAI: {e}')
            return 'STOP'  # Safe default

    def perform_scan(self):
        """Perform a scanning sequence to capture all three views."""
        if self.latest_frame is None:
            self.get_logger().warn("No image available for scanning")
            return False

        if self.scan_state == 0:
            # Starting scan
            self.get_logger().info("Starting scan sequence")
            self.scan_in_progress = True
            self.scan_state = 1
            return False

        elif self.scan_state == 1:
            # Capture LEFT view
            self.get_logger().info("Capturing LEFT view")
            self.send_servo_command("LEFT")
            with self.frame_lock:
                if self.latest_frame is not None:
                    self.left_image = self.latest_frame.copy()
            self.scan_state = 2
            return False

        elif self.scan_state == 2:
            # Capture FRONT view
            self.get_logger().info("Capturing FRONT view")
            self.send_servo_command("FRONT")
            with self.frame_lock:
                if self.latest_frame is not None:
                    self.front_image = self.latest_frame.copy()
            self.scan_state = 3
            return False

        elif self.scan_state == 3:
            # Capture RIGHT view
            self.get_logger().info("Capturing RIGHT view")
            self.send_servo_command("RIGHT")
            with self.frame_lock:
                if self.latest_frame is not None:
                    self.right_image = self.latest_frame.copy()
            self.scan_state = 4
            return False

        elif self.scan_state == 4:
            # Return to FRONT and complete scan
            self.get_logger().info("Returning to FRONT view")
            self.send_servo_command("FRONT")
            self.scan_state = 0
            self.scan_in_progress = False
            self.last_scan_time = time.time()

            # Verify all images were captured
            if self.left_image is None or self.front_image is None or self.right_image is None:
                self.get_logger().error("Failed to capture all views during scan")
                return False

            self.get_logger().info("Scan sequence completed successfully")
            return True

        return False

    def navigation_loop(self):
        """Main loop for navigation with improved timing."""
        if self.is_arrived:
            # We've arrived at the destination - just keep sending stop
            self.cmd_vel_pub.publish(Twist())
            return

        current_time = time.time()

        # Check if we need to start a scan
        if not self.scan_in_progress:
            # Start a scan if:
            # 1. We're not currently moving, OR
            # 2. We're in continuous mode and it's been a while since last scan
            if (not self.currently_moving or
                (self.continuous_movement and current_time - self.last_scan_time > self.scan_interval)):
                self.scan_in_progress = True
                self.scan_state = 0

        # Handle scanning
        if self.scan_in_progress:
            scan_complete = self.perform_scan()

            # If scan complete, process the images
            if scan_complete:
                # Get command from vision model with all three views
                # Choose one: self.query_openai or self.query_gemini
                command = self.query_openai(self.left_image, self.front_image, self.right_image)
                self.get_logger().info(f'Vision model response: {command}')

                # Check if we should stop
                if "STOP" in command:
                    self.is_arrived = True
                    self.send_movement_command("STOP")
                    self.get_logger().info('Destination reached!')
                    return

                # Send the movement command
                self.send_movement_command(command)

        # Handle continuous movement behavior
        if self.currently_moving:
            # Check if we've been moving long enough
            if current_time - self.movement_start_time >= self.min_cmd_duration:
                if not self.continuous_movement:
                    # If not in continuous mode, stop after min duration
                    self.send_movement_command("STOP")
                    self.get_logger().info('Stopped after minimum movement duration')
                # Otherwise, keep moving until next scan

    def __del__(self):
        """Clean up when the node is destroyed."""
        if self.debug_socket:
            try:
                self.debug_socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = DebugNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('Stopping robot and shutting down')
        # Clean up debug socket
        if node.debug_socket:
            try:
                node.debug_socket.close()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
