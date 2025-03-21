# Multi-View Robot Navigation System

This repository contains code for a ROS2-based robot navigation system that uses vision models to navigate in indoor environments. The system uses a Raspberry Pi with a movable camera (via servo motor) to capture multiple views (left, front, right) and then uses OpenAI or Gemini vision models to determine the next movement.

## System Architecture

The system is split between two computers:

1. **Raspberry Pi**: Controls the hardware (motors, servo) and publishes camera images
2. **Laptop**: Processes images, communicates with AI models, and sends commands to the robot

### Components

- **Raspberry Pi**:
  - `raspi_robot_controller.py`: Unified controller for motors and servo
  - `ros2 run image_tools cam2image`: ROS2 tool to publish camera images

- **Laptop**:
  - `improved_navigation_node_debug.py`: Main navigation node that communicates with AI models
  - `run_debug.py`: Script to launch and configure the navigation node
  - `conversation_tracker.py`: Tracks and displays conversations with the AI model
  - `navigation_debug.py`: Monitors robot state and visualizes camera feed

## Setup Instructions

### Requirements

- ROS2 Jazzy installed on both Raspberry Pi and laptop
- Python 3.10+ on both systems
- Network connection between Raspberry Pi and laptop

### Installation

1. **Clone this repository**:
   ```bash
   git clone https://github.com/yourusername/multi-view-navigation.git
   cd multi-view-navigation
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **On the Raspberry Pi**:
   ```bash
   # Install additional hardware dependencies
   pip install RPi.GPIO adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit board
   ```

### Network Setup

For ROS2 to work across multiple machines:

1. **Set up ROS_DOMAIN_ID** (same on both machines):
   ```bash
   export ROS_DOMAIN_ID=42  # Choose any number between 0-232
   ```

2. **Configure network**:
   - Both machines should be on the same network
   - Each machine should be able to ping the other

## Running the System

### 1. Start ROS2 on the Raspberry Pi

```bash
# Terminal 1: Start the camera publisher
ros2 run image_tools cam2image

# Terminal 2: Start the robot controller
cd ~/path/to/repository/raspberry_pi
python3 raspi_robot_controller.py
```

### 2. Start the navigation system on the laptop

```bash
# Terminal 1: Start the conversation tracker
cd ~/path/to/repository/laptop
python3 conversation_tracker.py

# Terminal 2: Start the visual debug tool
cd ~/path/to/repository/laptop
python3 navigation_debug.py

# Terminal 3: Start the navigation node
cd ~/path/to/repository/laptop
python3 run_debug.py --target "go to the kitchen" --continuous
```

### Command-line Arguments

The `run_debug.py` script accepts these arguments:

- `--target`: Destination for the robot (e.g., "go to the kitchen")
- `--model`: Vision model to use (`gpt-4o` or `gemini-2.0-flash`)
- `--servo-delay`: Delay after moving servo (seconds)
- `--min-cmd-duration`: Minimum duration for movement commands (seconds)
- `--continuous`: Enable continuous movement mode
- `--scan-interval`: In continuous mode, how often to scan (seconds)
- `--debug-port`: Port for communication with debug tools

## Environment Variables

Set these environment variables before running:

- `OPENAI_API_KEY`: Your OpenAI API key (required if using OpenAI models)
- `GEMINI_API_KEY`: Your Google Gemini API key (required if using Gemini models)

Example:
```bash
export OPENAI_API_KEY="your-api-key-here"
export GEMINI_API_KEY="your-gemini-api-key-here"
```

## Troubleshooting

- **Camera not working**: Check if the camera is properly connected and if the camera publisher is running
- **Robot not moving**: Verify that the motor controller is receiving commands
- **API errors**: Check that your API keys are correctly set

## Notes

- The system uses the OpenAI base URL redirected to Google's API for Gemini models: `https://generativelanguage.googleapis.com/v1beta/openai/`
- For debugging, run each component in a separate terminal so you can see all the outputs