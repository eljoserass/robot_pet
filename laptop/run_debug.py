#!/usr/bin/env python3
"""
Launch script for the debug-enabled multi-view navigation system.
"""
import rclpy
import os
import sys
import argparse
from improved_navigation_node_debug import DebugNavigationNode

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Launch the debug-enabled navigation system')
    parser.add_argument('--target', type=str, default='go to the kitchen',
                        help='Target location to navigate to')
    parser.add_argument('--model', type=str, default='gpt-4o',
                        help='Model to use for vision processing (gpt-4o or gemini-2.0-flash)')
    parser.add_argument('--servo-delay', type=float, default=0.3,
                        help='Delay after moving servo (seconds)')
    parser.add_argument('--min-cmd-duration', type=float, default=0.3,
                        help='Minimum duration for each movement command (seconds)')
    parser.add_argument('--continuous', action='store_true', default=False,
                        help='Enable continuous movement (instead of stop-and-go)')
    parser.add_argument('--scan-interval', type=float, default=3.0,
                        help='In continuous mode, how often to scan for new directions (seconds)')
    parser.add_argument('--debug-port', type=int, default=9876,
                        help='Port for communication with debug tools')
    args = parser.parse_args()

    # Make sure we have the required API keys
    if 'OPENAI_API_KEY' not in os.environ:
        print("WARNING: OPENAI_API_KEY environment variable not set!")
        api_key = input("Enter your OpenAI API key (or leave empty to exit): ")
        if not api_key:
            print("No API key provided. Exiting.")
            sys.exit(1)
        os.environ['OPENAI_API_KEY'] = api_key
        
    if 'GEMINI_API_KEY' not in os.environ:
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        api_key = input("Enter your Gemini API key (or leave empty to use default): ")
        if api_key:
            os.environ['GEMINI_API_KEY'] = api_key

    # Initialize ROS2
    rclpy.init()

    # Create and configure the navigation node
    node = DebugNavigationNode()

    # Update node parameters from command line arguments
    node.target_location = args.target
    node.model_name = args.model
    node.servo_delay = args.servo_delay
    node.min_cmd_duration = args.min_cmd_duration
    node.continuous_movement = args.continuous
    node.scan_interval = args.scan_interval
    node.debug_port = args.debug_port

    print(f"Debug-enabled navigation started with target: {args.target}")
    print(f"Using model: {args.model}")
    print(f"Continuous movement: {args.continuous}")
    print(f"Debug port: {args.debug_port}")
    print()
    print("Remember to run these debug tools in separate terminals:")
    print("1. python3 conversation_tracker.py  # To see LLM conversation")
    print("2. python3 navigation_debug.py      # To monitor robot behavior")

    try:
        # Run the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Navigation stopped by user")
    except Exception as e:
        print(f"Error during navigation: {e}")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
