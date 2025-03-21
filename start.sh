#!/bin/bash
# Main launch script for the multi-view navigation system

echo "==================================================="
echo "Multi-View Navigation System Launcher"
echo "==================================================="

# Check if we're on the Raspberry Pi or laptop
if [ -f /etc/rpi-issue ]; then
    echo "Detected Raspberry Pi environment"
    echo "This script will help start the robot controller."
    
    # Set environment
    export ROS_DOMAIN_ID=42  # Choose a number between 0-232
    
    echo "1. Start the standard ROS2 camera publisher with:"
    echo "   ros2 run image_tools cam2image"
    echo ""
    echo "2. OR use our custom camera publisher:"
    echo "   python3 raspberry_pi/camera_publisher.py"
    echo ""
    echo "3. Start the robot controller with:"
    echo "   python3 raspberry_pi/raspi_robot_controller.py"
    
else
    echo "Detected laptop/desktop environment"
    echo "This script will help set up and start the navigation system."
    
    # Set environment
    export ROS_DOMAIN_ID=42  # Choose a number between 0-232
    
    # Create needed directories
    cd laptop
    bash create_folders.sh
    cd ..
    
    # Check for API keys
    if [ -z "$OPENAI_API_KEY" ]; then
        echo "WARNING: OPENAI_API_KEY is not set!"
        echo "Please set it with: export OPENAI_API_KEY=\"your-key-here\""
    else
        echo "✓ OPENAI_API_KEY is set"
    fi
    
    if [ -z "$GEMINI_API_KEY" ]; then
        echo "WARNING: GEMINI_API_KEY is not set!"
        echo "Please set it with: export GEMINI_API_KEY=\"your-key-here\""
    else
        echo "✓ GEMINI_API_KEY is set"
    fi
    
    echo ""
    echo "Start these in separate terminals:"
    echo ""
    echo "1. Start the conversation tracker:"
    echo "   cd laptop && python3 conversation_tracker.py"
    echo ""
    echo "2. Start the navigation debugger:"
    echo "   cd laptop && python3 navigation_debug.py"
    echo ""
    echo "3. Start the navigation system:"
    echo "   cd laptop && python3 run_debug.py --target \"go to the kitchen\" --continuous"
    echo ""
fi

echo "==================================================="
echo "Happy navigating!"
echo "==================================================="