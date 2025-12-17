---
sidebar_position: 1
---

# Chapter 1: ROS 2 Basics for Physical AI

## Introduction to ROS 2

Welcome to the ROS 2 Robotics Module! This chapter will introduce you to the fundamental concepts of ROS 2 (Robot Operating System 2), which serves as the middleware connecting AI logic to humanoid robot control.

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## What is ROS 2?

ROS 2 is the next generation of the Robot Operating System, designed to address the needs of commercial, professional, and academic robotics applications. It provides:

- A distributed computing framework for robot systems
- A package management system for robot software
- Tools for building, testing, and deploying robot applications
- A rich ecosystem of existing packages and libraries

## ROS 2 as a Distributed Robotic Nervous System

Think of ROS 2 as the nervous system of a robot. Just as the nervous system connects different parts of a biological organism and allows them to communicate, ROS 2 connects different components of a robot system and enables them to work together.

In this "nervous system":
- Nodes are like different organs or subsystems
- Topics are like nerves that carry sensory and motor information
- Services are like reflexes that respond to specific requests
- Actions are like complex behaviors that take time and provide feedback

This architecture allows AI logic to communicate with robot control systems seamlessly, enabling sophisticated robot behaviors.

## Core Communication Concepts

ROS 2 provides four primary forms of inter-process communication:

1. **Nodes**: The fundamental building blocks of ROS 2
2. **Topics**: For asynchronous publish-subscribe communication
3. **Services**: For synchronous request-response communication
4. **Actions**: For goal-oriented communication with feedback

These concepts form the backbone of how different parts of a robot system communicate with each other.

## ROS 2 Node Concept

A **Node** is the fundamental building block of a ROS 2 system. It's a process that performs computation and communicates with other nodes in the system. In the context of AI and robotics, nodes can represent different components such as:

- Sensor processing nodes
- Control algorithm nodes
- AI decision-making nodes
- Hardware interface nodes

### Node Definition and Attributes

Each ROS 2 Node has the following key attributes:

- **node_name**: A unique string identifier for the node within its namespace
- **namespace**: An optional grouping mechanism that allows for logical organization of nodes
- **lifecycle**: Nodes have different states (Created, Active, Inactive, Finalized) that manage their execution

### Node Relationships

Nodes communicate with other nodes through topics, services, and actions. They form the foundation of the distributed architecture, where multiple nodes can run on different machines but still communicate as part of the same system.

### Basic Node Structure in Python

Here's a minimal example of a ROS 2 node in Python using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    # Keep the node running
    rclpy.spin(minimal_node)

    # Clean up
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates the basic structure of a ROS 2 node with initialization, logging, and proper cleanup.

## ROS 2 Topic Concept

A **Topic** is a named bus over which nodes exchange messages in a publish-subscribe pattern. This is the most common form of communication in ROS 2 and is used for continuous data streams like sensor readings, robot state information, or control commands.

### Publish-Subscribe Pattern

The publish-subscribe pattern works as follows:
- **Publishers**: Nodes that send messages to a topic
- **Subscribers**: Nodes that receive messages from a topic
- **Messages**: Data structures that carry information between nodes

Multiple publishers can send messages to the same topic, and multiple subscribers can receive messages from the same topic. This creates a decoupled system where publishers don't need to know who is listening, and subscribers don't need to know who is publishing.

### Topic Attributes

Each ROS 2 Topic has the following key attributes:

- **topic_name**: A unique string identifier for the topic
- **message_type**: Defines the structure of messages sent on the topic (e.g., sensor_msgs.msg.LaserScan, std_msgs.msg.String)
- **QoS (Quality of Service) profile**: Settings that control reliability, durability, and other communication characteristics

### Practical Example: Publisher and Subscriber

Here's a simple example showing how to create a publisher and subscriber in Python:

**Publisher:**
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

**Subscriber:**
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

This example demonstrates the publish-subscribe pattern where one node publishes messages and another subscribes to them.

## ROS 2 Service Concept

A **Service** is a request-response communication pattern between nodes for synchronous operations. Unlike topics which provide continuous data streams, services provide a way for nodes to request specific information or ask another node to perform a specific action and wait for a response.

### Request-Response Pattern

The request-response pattern works as follows:
- **Service Client**: A node that sends a request to a service
- **Service Server**: A node that receives the request, processes it, and sends back a response
- **Request/Response Messages**: Structured data that defines the format of requests and responses

This pattern is useful for operations that have a clear beginning and end, such as requesting sensor calibration, asking for the current robot pose, or commanding a specific action.

### Service Attributes

Each ROS 2 Service has the following key attributes:

- **service_name**: A unique string identifier for the service
- **request_type**: Defines the structure of the request message
- **response_type**: Defines the structure of the response message
- **QoS (Quality of Service) profile**: Settings that control reliability and other communication characteristics

### Practical Example: Service Client and Server

Here's a simple example showing how to create a service client and server in Python:

**Service Server:**
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
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    future = minimal_client.send_request(1, 2)

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

This example demonstrates the request-response pattern where one node provides a service to add two integers, and another node calls that service.

## ROS 2 Action Concept

An **Action** is a goal-oriented communication pattern with feedback and status updates. Actions are used for long-running tasks that might take a significant amount of time to complete, such as navigation to a specific location, manipulation of objects, or complex robot behaviors.

### Goal-Feedback-Result Pattern

The action pattern works as follows:
- **Action Client**: A node that sends a goal to an action server
- **Action Server**: A node that accepts goals, provides feedback during execution, and returns a result when complete
- **Goal**: The request to perform a specific task
- **Feedback**: Periodic updates on the progress of the goal
- **Result**: The final outcome of the goal execution

### Action Attributes

Each ROS 2 Action has the following key attributes:

- **action_name**: A unique string identifier for the action
- **goal_type**: Defines the structure of the goal message
- **feedback_type**: Defines the structure of the feedback message
- **result_type**: Defines the structure of the result message

### Practical Example: Action Client and Server

Here's a simple example showing how to create an action client and server in Python:

**Action Server:**
```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Returning result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    finally:
        fibonacci_action_server.destroy()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client:**
```python
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

This example demonstrates the goal-feedback-result pattern where a client sends a goal to calculate a Fibonacci sequence, receives feedback during the calculation, and gets the final result.

## Target Audience

This module is designed for AI students with basic Python knowledge who are new to ROS 2. We'll focus on practical applications of ROS 2 concepts for connecting AI logic to robot control systems, particularly in the context of humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the roles of nodes, topics, services, and actions in ROS 2
- Understand how ROS 2 components communicate in a distributed architecture
- Identify these components in a simple robot system diagram
- Understand the basic structure of a ROS 2 application

## Exercises

### Exercise 1: Identifying ROS 2 Components

Consider a simple robot system with the following components:
- A camera node that publishes images
- A navigation node that plans paths
- A motor control node that moves the robot
- A battery monitoring node that checks power levels
- A speech recognition node that processes voice commands

For each of the following scenarios, identify which ROS 2 communication pattern (topic, service, or action) would be most appropriate and explain why:

1. The camera node needs to continuously send images to a computer vision node for processing.
2. The navigation node needs to request the current robot position from the localization node.
3. The speech recognition node needs to command the robot to move to a specific location, which could take several minutes.
4. The battery monitoring node needs to continuously broadcast the current battery level to all other nodes.

### Exercise 2: Node Communication Analysis

In the same robot system described above, identify potential nodes and their communication patterns for the following tasks:

1. Design a communication architecture where the robot can respond to voice commands to navigate to specific locations.
2. Describe how sensor data from multiple sources (camera, lidar, IMU) would be shared with the navigation system.
3. Explain how the robot could request permission from a central system before executing critical operations.

### Exercise 3: Code Analysis

Analyze the example code provided in this chapter and answer the following questions:

1. What is the purpose of the `rclpy.init()` function in the node examples?
2. How does a publisher node differ from a subscriber node in terms of setup and functionality?
3. What is the role of the `QoS` profile in topic communication?
4. Why do service clients need to wait for the service to be available before making a request?
5. What are the key differences between services and actions in terms of communication patterns?

### Exercise 4: Scenario-Based Questions

Consider a humanoid robot designed to assist in a home environment:

1. Identify at least 5 different nodes that would be needed for the robot to perform basic household tasks (e.g., picking up objects, navigating rooms, responding to voice commands).
2. For each node identified, specify what topics it would likely publish or subscribe to.
3. Describe what services this robot might provide to a user interface application.
4. What actions would be appropriate for complex tasks like grasping an object or walking to a specific location?