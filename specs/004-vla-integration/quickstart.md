# Quickstart: Vision-Language-Action (VLA) Robotics Module

## Overview
This quickstart guide provides a rapid introduction to the Vision-Language-Action (VLA) concepts and how to implement a basic VLA pipeline for humanoid robots using simulation.

## Prerequisites

- Basic understanding of ROS 2 (covered in Module 1)
- Familiarity with simulation environments (covered in Module 2)
- Understanding of AI/robotics concepts (covered in Module 3)

## Setting Up the VLA Environment

### 1. Install Required Dependencies

```bash
# Ensure ROS 2 Humble is installed and sourced
source /opt/ros/humble/setup.bash

# Install Python dependencies for speech recognition
pip install speechrecognition pyaudio openai

# For simulation environment
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-bridge
```

### 2. Create the VLA Workspace

```bash
mkdir -p ~/vla_ws/src
cd ~/vla_ws
colcon build
source install/setup.bash
```

## Basic VLA Pipeline Implementation

### 1. Voice-to-Action Pipeline

Create a basic voice recognition node:

```python
# voice_to_action_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Start listening thread
        self.voice_thread = threading.Thread(target=self.listen_for_commands)
        self.voice_thread.daemon = True
        self.voice_thread.start()

        self.get_logger().info("Voice-to-Action node initialized")

    def listen_for_commands(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0)

                # Recognize speech
                command = self.recognizer.recognize_google(audio).lower()
                self.process_command(command)

            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                self.get_logger().info("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Recognition error: {e}")

    def process_command(self, command):
        """Process voice command and generate robot action"""
        msg = Twist()

        if "forward" in command or "move forward" in command:
            msg.linear.x = 0.5
        elif "backward" in command or "move backward" in command:
            msg.linear.x = -0.5
        elif "left" in command or "turn left" in command:
            msg.angular.z = 0.5
        elif "right" in command or "turn right" in command:
            msg.angular.z = -0.5
        elif "stop" in command:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.get_logger().info(f"Unknown command: {command}")
            return

        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Executed command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Run the Basic Pipeline

```bash
# Terminal 1: Start your simulated robot
ros2 launch your_robot_gazebo your_robot_world.launch.py

# Terminal 2: Run the voice-to-action node
python3 voice_to_action_node.py

# Speak commands like "move forward" or "turn left" to control the robot
```

## LLM-Based Cognitive Planning Example

### 1. Simple LLM Integration

```python
# llm_planner_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Subscriber for natural language commands
        self.command_sub = self.create_subscription(
            String, 'natural_language_command', self.command_callback, 10)

        # Publisher for action sequences
        self.action_pub = self.create_publisher(String, 'action_sequence', 10)

        # Configure OpenAI API (use your own key)
        # openai.api_key = "your-api-key-here"

        self.get_logger().info("LLM Planner node initialized")

    def command_callback(self, msg):
        """Process natural language command through LLM"""
        command = msg.data

        # In practice, you'd call the LLM API here
        # For this example, we'll simulate a simple planner
        action_sequence = self.plan_actions(command)

        # Publish the action sequence
        action_msg = String()
        action_msg.data = json.dumps(action_sequence)
        self.action_pub.publish(action_msg)

        self.get_logger().info(f"Planned actions for: {command}")

    def plan_actions(self, command):
        """Simple action planner (in practice, use LLM)"""
        if "bring me the cup" in command:
            return [
                {"action": "navigate", "params": {"location": "kitchen"}},
                {"action": "locate", "params": {"object": "cup"}},
                {"action": "grasp", "params": {"object": "cup"}},
                {"action": "navigate", "params": {"location": "user"}},
                {"action": "place", "params": {"location": "user"}}
            ]
        elif "go to kitchen" in command:
            return [
                {"action": "navigate", "params": {"location": "kitchen"}}
            ]
        else:
            return [{"action": "unknown", "params": {"command": command}}]

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete VLA Integration Example

### 1. System Architecture

The complete VLA system integrates:

1. **Voice Recognition**: Converts speech to text
2. **LLM Planning**: Converts text to action sequences
3. **Action Execution**: Executes robot actions in simulation
4. **Feedback Loop**: Provides status updates

### 2. Running the Complete System

```bash
# Terminal 1: Launch simulation
ros2 launch your_humanoid_gazebo humanoid_world.launch.py

# Terminal 2: Start voice recognition
python3 voice_to_action_node.py

# Terminal 3: Start LLM planner
python3 llm_planner_node.py

# Terminal 4: Start action executor
ros2 run your_robot_control action_executor_node
```

## Testing Your VLA System

### 1. Basic Voice Commands
- "Move forward 1 meter"
- "Turn left"
- "Stop"
- "Go to the kitchen"

### 2. Complex Natural Language Commands
- "Please bring me the red cup from the table"
- "Navigate to the living room and wait there"
- "Pick up the book and place it on the shelf"

## Next Steps

1. **Chapter 1**: Deep dive into voice-to-action pipelines
2. **Chapter 2**: Explore cognitive planning with LLMs
3. **Chapter 3**: Build complete autonomous humanoid systems
4. **Summary**: Integrate all components and explore advanced topics

## Troubleshooting

- **No voice recognition**: Check microphone permissions and audio drivers
- **LLM not responding**: Verify API key and network connectivity
- **Robot not moving**: Check ROS 2 network configuration and topic connections
- **Poor recognition accuracy**: Train speech models with domain-specific vocabulary