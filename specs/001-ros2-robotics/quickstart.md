# Quickstart Guide: ROS 2 Robotics Module

## Overview
This quickstart guide provides a minimal example to help you get started with the ROS 2 Robotics Module. It covers the basic setup needed to run the examples in the documentation.

## Prerequisites
- Basic Python knowledge
- ROS 2 installed (Humble Hawksbill or later recommended)
- Python 3.8 or later
- Docusaurus development environment

## Setting Up Your Environment

### 1. Install ROS 2
If you haven't already installed ROS 2, follow the official installation guide for your platform:
- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### 2. Verify Installation
```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 installation path
ros2 --version
```

### 3. Create a Workspace
```bash
mkdir -p ~/ros2_book_ws/src
cd ~/ros2_book_ws
```

## Running Your First Example

### 1. Create a Simple Publisher Node
Create a file named `simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Create a Simple Subscriber Node
Create a file named `simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Run the Example
Open two terminal windows:

Terminal 1:
```bash
cd ~/ros2_book_ws
source /opt/ros/humble/setup.bash
python3 simple_publisher.py
```

Terminal 2:
```bash
cd ~/ros2_book_ws
source /opt/ros/humble/setup.bash
python3 simple_subscriber.py
```

You should see the publisher sending messages and the subscriber receiving them.

## Docusaurus Documentation Setup

### 1. Install Node.js Dependencies
```bash
cd ~/path-to-your-book-ai-repo
npm install
```

### 2. Start the Documentation Server
```bash
npm run start
```

The documentation will be available at http://localhost:3000

## Next Steps
- Read Chapter 1: ROS 2 Basics for Physical AI to understand the concepts behind these examples
- Explore the rclpy documentation to learn more about Python ROS 2 development
- Try modifying the examples to understand how nodes, topics, and messages work together