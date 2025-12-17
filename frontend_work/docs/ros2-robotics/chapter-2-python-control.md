---
sidebar_position: 2
---

# Chapter 2: Python Control with rclpy

## Introduction to rclpy

Welcome to Chapter 2 of the ROS 2 Robotics Module! In this chapter, we'll explore how to implement ROS 2 nodes using Python through the `rclpy` library, which is the Python client library for ROS 2. This is essential for connecting AI agents to ROS controllers.

`rclpy` provides a Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, make service calls, and execute actions. It's designed to be intuitive for Python developers while maintaining the robustness and flexibility of the ROS 2 framework.

## Why Python for ROS 2?

Python is an excellent choice for developing ROS 2 applications, particularly when connecting AI logic to robot control systems:

- **Ease of Use**: Python's simple syntax allows for rapid prototyping and development
- **AI Integration**: Python has extensive libraries for AI and machine learning (TensorFlow, PyTorch, scikit-learn)
- **Robotics Libraries**: Many robotics algorithms and tools have Python interfaces
- **Cross-platform**: Python code runs consistently across different operating systems
- **Community Support**: Large Python community with extensive resources and packages

## Installing and Setting Up rclpy

Before we dive into coding, ensure you have ROS 2 installed with Python support. The `rclpy` library is typically included with ROS 2 distributions like Humble Hawksbill or later.

To verify your installation, you can import rclpy in a Python script:

```python
import rclpy
print("rclpy version:", rclpy.get_rclpy_version_string())
```

## Basic Node Structure with rclpy

Every ROS 2 Python node follows a standard structure that includes initialization, main logic, and cleanup. Here's the basic template:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize your node's components here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

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

This structure ensures proper initialization and cleanup of the node, which is important for robust robot applications.

## Detailed Node Implementation

Let's look at a more comprehensive example of a ROS 2 node implemented with rclpy. This example demonstrates how to create a node that performs a specific function:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create a publisher for sending velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber for laser scan data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Create a publisher for status messages
        self.status_publisher = self.create_publisher(String, 'status', 10)

        # Timer for periodic behavior
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.obstacle_distance = float('inf')
        self.get_logger().info('Robot Controller Node initialized')

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        if len(msg.ranges) > 0:
            # Get the minimum distance (simplified for front direction)
            self.obstacle_distance = min(msg.ranges)
            self.get_logger().debug(f'Detected obstacle at {self.obstacle_distance:.2f}m')

    def control_loop(self):
        """Main control loop executed periodically"""
        cmd = Twist()

        # Simple obstacle avoidance logic
        if self.obstacle_distance < 1.0:  # If obstacle closer than 1 meter
            cmd.linear.x = 0.0  # Stop moving forward
            cmd.angular.z = 0.5  # Turn right
            self.get_logger().info('Obstacle detected! Turning right.')
        else:
            cmd.linear.x = 0.2  # Move forward slowly
            cmd.angular.z = 0.0  # No turning
            self.get_logger().info('Clear path, moving forward.')

        # Publish the velocity command
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControllerNode()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info('Shutting down Robot Controller Node...')
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates several key concepts:

1. **Node Initialization**: Properly calling the parent constructor with a unique node name
2. **Publisher Creation**: Creating publishers to send messages to topics
3. **Subscriber Creation**: Creating subscribers to receive messages from topics
4. **Timer Usage**: Using timers for periodic execution of control logic
5. **Logging**: Using the built-in logger for debugging and status information
6. **Error Handling**: Properly handling shutdown procedures

## Advanced Node Features

ROS 2 nodes can also include more advanced features like parameters, which allow configuration without recompiling:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.5)
        self.declare_parameter('safety_distance', 1.0)

        # Get parameter values
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Create publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(
            f'Configurable Node initialized with: '
            f'linear_vel={self.linear_velocity}, '
            f'angular_vel={self.angular_velocity}, '
            f'safety_dist={self.safety_distance}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()

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

Parameters can be set at launch time or changed during runtime, making nodes more flexible and configurable.

## Publisher Implementation with rclpy

Publishers are used to send messages to topics in the ROS 2 system. Creating a publisher in rclpy involves calling the `create_publisher()` method on a node instance. Here's the basic syntax:

```python
publisher = node.create_publisher(message_type, topic_name, qos_profile_or_history_depth)
```

### Basic Publisher Example

Here's a simple publisher that sends string messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
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
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher with Custom Message Types

Publishers can send various types of messages, not just strings. Here's an example that publishes geometry messages for robot control:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class RobotCommandPublisher(Node):
    def __init__(self):
        super().__init__('robot_command_publisher')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for sensor values
        self.sensor_publisher = self.create_publisher(Float32, 'sensor_value', 10)

        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.i = 0

    def publish_commands(self):
        # Create and publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_msg.angular.z = 0.2  # Turn at 0.2 rad/s
        self.cmd_vel_publisher.publish(cmd_msg)

        # Create and publish sensor value
        sensor_msg = Float32()
        sensor_msg.data = 42.5  # Example sensor value
        self.sensor_publisher.publish(sensor_msg)

        self.get_logger().info(f'Published velocity command: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    publisher = RobotCommandPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down publisher...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher with Quality of Service (QoS) Settings

QoS settings allow you to configure how messages are delivered. Here's an example with custom QoS settings:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')

        # Create a QoS profile with specific settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure all messages are delivered
            durability=DurabilityPolicy.VOLATILE,    # Don't keep messages for late-joining subscribers
        )

        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'QoS message {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published with QoS: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = QoSPublisher()

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

## Subscriber Implementation with rclpy

Subscribers are used to receive messages from topics in the ROS 2 system. Creating a subscriber in rclpy involves calling the `create_subscription()` method on a node instance. Here's the basic syntax:

```python
subscription = node.create_subscription(
    message_type,
    topic_name,
    callback_function,
    qos_profile_or_history_depth
)
```

### Basic Subscriber Example

Here's a simple subscriber that receives string messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
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
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Multiple Subscribers in One Node

A single node can subscribe to multiple topics simultaneously:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        # Subscribe to different message types
        self.string_sub = self.create_subscription(
            String,
            'status',
            self.string_callback,
            10
        )

        self.float_sub = self.create_subscription(
            Float32,
            'sensor_value',
            self.float_callback,
            10
        )

        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        self.latest_status = "Unknown"
        self.latest_sensor = 0.0
        self.latest_cmd = None

    def string_callback(self, msg):
        self.latest_status = msg.data
        self.get_logger().info(f'Status update: {msg.data}')

    def float_callback(self, msg):
        self.latest_sensor = msg.data
        self.get_logger().info(f'Sensor value: {msg.data}')

    def twist_callback(self, msg):
        self.latest_cmd = msg
        self.get_logger().info(f'Command received: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down multi-subscriber node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber with Custom QoS Settings

Like publishers, subscribers can also use custom QoS profiles:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')

        # Create a QoS profile with specific settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure all messages are received
            durability=DurabilityPolicy.VOLATILE,    # Don't keep old messages
        )

        self.subscription = self.create_subscription(
            String,
            'qos_topic',
            self.qos_callback,
            qos_profile
        )

    def qos_callback(self, msg):
        self.get_logger().info(f'Received with QoS: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = QoSSubscriber()

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

## Service Client Implementation with rclpy

Service clients are used to make requests to services in the ROS 2 system. Creating a service client in rclpy involves calling the `create_client()` method on a node instance. Here's the basic syntax:

```python
client = node.create_client(service_type, service_name)
```

### Basic Service Client Example

Here's a simple service client that calls an "add two integers" service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Make an asynchronous service call
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send a request
    future = minimal_client.send_request(1, 2)

    # Spin until the future is complete
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if future.done():
            try:
                response = future.result()
                minimal_client.get_logger().info(f'Result: {response.sum}')
            except Exception as e:
                minimal_client.get_logger().info(f'Service call failed: {e}')
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client with Callbacks

For better responsiveness, you can use callbacks instead of blocking:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from example_interfaces.srv import AddTwoInts

class CallbackClient(Node):
    def __init__(self):
        super().__init__('callback_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Create a timer to make service calls periodically
        self.timer = self.create_timer(2.0, self.make_service_call)
        self.request_counter = 0

    def make_service_call(self):
        # Wait for service to be available
        if self.cli.wait_for_service(timeout_sec=1.0):
            request = AddTwoInts.Request()
            request.a = self.request_counter
            request.b = self.request_counter + 1

            # Make asynchronous call with a callback
            future = self.cli.call_async(request)
            future.add_done_callback(self.service_callback)

            self.get_logger().info(f'Sent request: {request.a} + {request.b}')
            self.request_counter += 1
        else:
            self.get_logger().warn('Service not available')

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CallbackClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down callback client...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Server Implementation with rclpy

Service servers are used to provide services that can be called by clients in the ROS 2 system. Creating a service server in rclpy involves calling the `create_service()` method on a node instance. Here's the basic syntax:

```python
server = node.create_service(service_type, service_name, callback_function)
```

### Basic Service Server Example

Here's a simple service server that implements an "add two integers" service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}, sum: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Service Server with Custom Logic

Here's a more complex example that implements a service for controlling a robot's movement:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')

        # Create the service
        self.srv = self.create_service(
            Trigger,
            'robot_control',
            self.robot_control_callback
        )

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create publisher for robot status
        self.status_publisher = self.create_publisher(Bool, 'robot_enabled', 10)

        self.robot_enabled = False
        self.get_logger().info('Robot Control Service initialized')

    def robot_control_callback(self, request, response):
        # In a real implementation, you might parse the request to determine the action
        # For this example, we'll just toggle the robot's enabled state

        self.robot_enabled = not self.robot_enabled

        # Send a command to the robot
        cmd_msg = Twist()
        if self.robot_enabled:
            cmd_msg.linear.x = 0.5  # Move forward
            self.get_logger().info('Robot enabled - sending forward command')
        else:
            cmd_msg.linear.x = 0.0  # Stop
            self.get_logger().info('Robot disabled - sending stop command')

        self.cmd_vel_publisher.publish(cmd_msg)

        # Publish status
        status_msg = Bool()
        status_msg.data = self.robot_enabled
        self.status_publisher.publish(status_msg)

        # Set response
        response.success = True
        response.message = f'Robot {"enabled" if self.robot_enabled else "disabled"}'

        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = RobotControlService()

    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        service_node.get_logger().info('Shutting down robot control service...')
    finally:
        service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting AI Agents to ROS Controllers

One of the primary goals of this module is to show how AI agents can connect to and control robot systems through ROS 2. This typically involves:

1. **Data Ingestion**: AI agents receive sensor data from ROS topics
2. **Decision Making**: AI algorithms process the data and make decisions
3. **Command Output**: AI agents send commands to robot controllers via topics, services, or actions

Let's look at a practical example that demonstrates how an AI agent can be integrated with ROS 2 to control a robot.

### AI Agent Node Example

Here's an example of a node that implements a simple AI agent that processes sensor data and makes decisions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Create subscriber for sensor data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create publisher for AI status
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        # Internal state
        self.laser_data = None
        self.safe_to_move = True

        # AI parameters
        self.safety_distance = 0.8  # meters
        self.linear_speed = 0.3     # m/s
        self.angular_speed = 0.5    # rad/s

        self.get_logger().info('AI Agent Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = np.array(msg.ranges)

        # Remove invalid readings (inf, nan)
        valid_data = self.laser_data[np.isfinite(self.laser_data)]

        if len(valid_data) > 0:
            min_distance = np.min(valid_data)
            self.safe_to_move = min_distance > self.safety_distance

            self.get_logger().debug(f'Min distance: {min_distance:.2f}m, Safe: {self.safe_to_move}')
        else:
            self.safe_to_move = False  # If no valid data, assume unsafe

    def make_decision(self):
        """AI decision making process"""
        if self.laser_data is None:
            # If no sensor data, stop the robot
            return 0.0, 0.0  # linear, angular velocity

        if self.safe_to_move:
            # If safe, move forward
            linear_vel = self.linear_speed
            angular_vel = 0.0
            status_msg = "Moving forward - clear path"
        else:
            # If obstacle detected, turn right
            linear_vel = 0.0
            angular_vel = -self.angular_speed  # Negative for right turn
            status_msg = f"Turning right - obstacle at {np.min(self.laser_data[np.isfinite(self.laser_data)]):.2f}m"

        # Publish status
        status = String()
        status.data = status_msg
        self.status_publisher.publish(status)

        return linear_vel, angular_vel

    def run_ai_loop(self):
        """Main AI control loop"""
        while rclpy.ok():
            linear_vel, angular_vel = self.make_decision()

            # Create and publish command
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_vel
            cmd_msg.angular.z = angular_vel

            self.cmd_publisher.publish(cmd_msg)

            self.get_logger().info(f'AI Decision: linear={linear_vel:.2f}, angular={angular_vel:.2f}')

            # Sleep to control loop frequency
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        ai_agent.run_ai_loop()
    except KeyboardInterrupt:
        ai_agent.get_logger().info('Shutting down AI Agent Node...')
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        ai_agent.cmd_publisher.publish(stop_msg)
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integrating with Popular AI Libraries

To integrate with popular AI libraries like TensorFlow or PyTorch, you can import them in your ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

# Import AI libraries
import tensorflow as tf

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')

        # Create subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Load a pre-trained model (example)
        # self.model = tf.keras.models.load_model('path/to/your/model')

        self.get_logger().info('AI Vision Node initialized')

    def image_callback(self, msg):
        """Process camera images with AI"""
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process with AI model (example)
            # result = self.model.predict(np.expand_dims(cv_image, axis=0))

            # For this example, we'll just detect red color
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Calculate center of red objects
            if cv2.countNonZero(mask) > 1000:  # If significant red detected
                # Find centroid
                moments = cv2.moments(mask)
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    height, width = mask.shape
                    center_x = width / 2

                    # Generate command based on object position
                    cmd_msg = Twist()
                    cmd_msg.linear.x = 0.2  # Move forward

                    # Turn toward the object
                    if cx < center_x - 50:
                        cmd_msg.angular.z = 0.3  # Turn left
                    elif cx > center_x + 50:
                        cmd_msg.angular.z = -0.3  # Turn right
                    else:
                        cmd_msg.angular.z = 0.0  # Go straight

                    self.cmd_publisher.publish(cmd_msg)
                    self.get_logger().info(f'Red object detected at {cx}, commanding turn')
                else:
                    # No clear centroid, stop
                    cmd_msg = Twist()
                    self.cmd_publisher.publish(cmd_msg)
            else:
                # No red object detected, stop
                cmd_msg = Twist()
                self.cmd_publisher.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    ai_vision = AIVisionNode()

    try:
        rclpy.spin(ai_vision)
    except KeyboardInterrupt:
        ai_vision.get_logger().info('Shutting down AI Vision Node...')
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        ai_vision.cmd_publisher.publish(stop_msg)
        ai_vision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Working Example: AI-Powered Robot Controller

Here's a complete, runnable example that demonstrates how to connect an AI agent to a ROS 2 robot controller. This example implements a simple AI that navigates a robot while avoiding obstacles:

### File: ai_robot_controller.py

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import math

class AIRobotController(Node):
    def __init__(self):
        super().__init__('ai_robot_controller')

        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create publisher for status messages
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Internal state
        self.laser_ranges = None
        self.safe_to_move = True
        self.obstacle_detected = False

        # Robot parameters
        self.safety_distance = 0.8  # meters
        self.linear_speed = 0.3     # m/s
        self.angular_speed = 0.4    # rad/s
        self.min_turn_radius = 0.5  # meters

        # AI state machine
        self.state = 'FORWARD'  # FORWARD, TURNING, STOPPED
        self.state_timer = 0

        # Setup timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('AI Robot Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_ranges = np.array(msg.ranges)

        # Process the scan data
        if self.laser_ranges is not None and len(self.laser_ranges) > 0:
            # Get valid ranges (remove inf and nan values)
            valid_ranges = self.laser_ranges[np.isfinite(self.laser_ranges)]

            if len(valid_ranges) > 0:
                min_distance = np.min(valid_ranges)
                self.obstacle_detected = min_distance < self.safety_distance
                self.safe_to_move = not self.obstacle_detected

                # Log obstacle detection
                if self.obstacle_detected:
                    self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m')

    def control_loop(self):
        """Main AI control loop"""
        if self.laser_ranges is None:
            # If no sensor data, stop the robot
            cmd_msg = Twist()
            self.cmd_publisher.publish(cmd_msg)
            return

        # State machine for navigation
        cmd_msg = Twist()

        if self.state == 'FORWARD':
            if self.obstacle_detected:
                # Obstacle detected, start turning
                self.state = 'TURNING'
                self.state_timer = 0
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = self.angular_speed  # Turn left
                status_msg = String()
                status_msg.data = 'Turning to avoid obstacle'
                self.status_publisher.publish(status_msg)
            else:
                # Path is clear, move forward
                cmd_msg.linear.x = self.linear_speed
                cmd_msg.angular.z = 0.0
                status_msg = String()
                status_msg.data = 'Moving forward'
                self.status_publisher.publish(status_msg)

        elif self.state == 'TURNING':
            # Continue turning until path is clear ahead
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.angular_speed  # Keep turning left

            # Check if front is clear
            front_ranges = self.laser_ranges[330:30]  # -30 to +30 degrees
            if len(front_ranges) > 0:
                front_min = np.min(front_ranges[np.isfinite(front_ranges)])
                if front_min > self.safety_distance:
                    self.state = 'FORWARD'  # Path is clear, go forward
                    status_msg = String()
                    status_msg.data = 'Path clear, moving forward'
                    self.status_publisher.publish(status_msg)

            self.state_timer += 0.1  # Increment timer (0.1s per call)

            # Timeout to avoid getting stuck turning
            if self.state_timer > 5.0:  # 5 seconds
                self.state = 'FORWARD'
                status_msg = String()
                status_msg.data = 'Timeout, returning to forward motion'
                self.status_publisher.publish(status_msg)

        # Publish the command
        self.cmd_publisher.publish(cmd_msg)

        # Log current state
        self.get_logger().info(f'State: {self.state}, Linear: {cmd_msg.linear.x:.2f}, Angular: {cmd_msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = AIRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down AI Robot Controller...')
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        controller.cmd_publisher.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run the Example

1. **Save the code** to a file named `ai_robot_controller.py`

2. **Make sure you have ROS 2 installed** with Python support

3. **If running in a simulated environment** like Gazebo, make sure you have a robot with:
   - A laser scanner publishing to the `/scan` topic
   - A velocity command subscriber on the `/cmd_vel` topic

4. **Run the node**:
   ```bash
   python3 ai_robot_controller.py
   ```

### Example: Simple Publisher-Subscriber Pair

Here's a simple publisher and subscriber pair that you can run to test basic communication:

**publisher.py:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
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

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        simple_publisher.get_logger().info('Shutting down publisher...')
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**subscriber.py:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Shutting down subscriber...')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this example:
1. Open two terminal windows
2. In the first terminal: `python3 publisher.py`
3. In the second terminal: `python3 subscriber.py`
4. You should see the publisher sending messages and the subscriber receiving them

## Troubleshooting Common rclpy Issues

When working with rclpy, you may encounter several common issues. Here are solutions to the most frequent problems:

### 1. Module Not Found Errors

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**:
- Make sure ROS 2 is installed and sourced properly
- In your terminal, run: `source /opt/ros/humble/setup.bash` (or your ROS 2 distribution)
- Verify installation: `python3 -c "import rclpy; print(rclpy.__version__)"`

### 2. Node Name Conflicts

**Problem**: Multiple nodes with the same name cause conflicts

**Solution**:
- Always use unique node names
- Add random suffixes or timestamps for dynamic names
- Use parameters to allow name configuration

```python
import uuid

def main(args=None):
    rclpy.init(args=args)
    unique_name = f'my_node_{uuid.uuid4().hex[:8]}'
    node = MyNode(unique_name)
    # ... rest of code
```

### 3. Topic Connection Issues

**Problem**: Publishers and subscribers not connecting to topics

**Solution**:
- Check that topic names match exactly (including case)
- Verify that both nodes are on the same ROS domain
- Use `ros2 topic list` to see available topics
- Use `ros2 topic info <topic_name>` to see publishers/subscribers

### 4. QoS Profile Mismatch

**Problem**: Messages not being received due to incompatible QoS settings

**Solution**:
- Ensure publisher and subscriber QoS profiles are compatible
- Use default profiles unless specific requirements exist
- Common compatible combinations:
  - RELIABLE publisher with RELIABLE subscriber
  - BEST_EFFORT publisher with BEST_EFFORT subscriber

### 5. Threading Issues

**Problem**: Callbacks not executing properly or race conditions

**Solution**:
- Use appropriate executors for multi-threading:
```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

### 6. Memory Leaks

**Problem**: Long-running nodes consuming increasing memory

**Solution**:
- Always call `destroy_node()` in a finally block
- Use limited history depth for publishers/subscribers
- Avoid storing unlimited amounts of data in node memory

### 7. Import Issues with Message Types

**Problem**: `ModuleNotFoundError` for specific message types like `sensor_msgs.msg`

**Solution**:
- Make sure the required ROS packages are installed
- Install with: `sudo apt install ros-humble-sensor-msgs` (for Ubuntu)
- Check that the ROS environment is properly sourced

### 8. Permission and Access Issues

**Problem**: Nodes unable to communicate across different terminals or users

**Solution**:
- Ensure all terminals source the same ROS environment
- Check that RMW (ROS Middleware) settings are consistent
- Use the same ROS_DOMAIN_ID across all nodes

### Debugging Tips

1. **Enable verbose logging**:
```python
import rclpy
rclpy.logging.set_logger_level('my_node', rclpy.logging.LoggingSeverity.DEBUG)
```

2. **Use ROS 2 command line tools**:
   - `ros2 node list` - List active nodes
   - `ros2 topic list` - List active topics
   - `ros2 topic echo <topic_name>` - Monitor topic messages
   - `ros2 service list` - List available services

3. **Add proper error handling**:
```python
def main(args=None):
    try:
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
```

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement ROS 2 nodes using the rclpy library
- Create publishers, subscribers, services, and actions in Python
- Connect AI logic to ROS 2 robot control systems
- Troubleshoot common issues with rclpy applications
- Build complete working examples that demonstrate AI-ROS integration