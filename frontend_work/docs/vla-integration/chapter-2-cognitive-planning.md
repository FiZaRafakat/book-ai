---
sidebar_position: 2
---

# Chapter 2: Cognitive Planning with LLMs

## Overview

This chapter explores the cognitive layer of Vision-Language-Action (VLA) systems, focusing on how Large Language Models (LLMs) translate natural language instructions into complex robot action sequences. Students will learn about task decomposition, planning logic, and the integration of LLMs with ROS 2 systems to create intelligent robotic behaviors.

Cognitive planning represents the bridge between high-level human intent and low-level robot execution, enabling robots to understand and carry out complex, multi-step tasks based on natural language commands.

## Large Language Model Integration with Robotics

### Understanding LLMs in Robotics Context

Large Language Models have revolutionized how robots can interpret and respond to human commands. Unlike traditional rule-based systems, LLMs can understand context, handle ambiguity, and generate appropriate responses to novel commands within their training domain.

In robotics applications, LLMs serve as the cognitive layer that:
- Interprets natural language commands
- Decomposes complex tasks into executable steps
- Maintains context across multiple interactions
- Adapts to new situations using learned patterns

### Key LLM Capabilities for Robotics

LLMs bring several valuable capabilities to robotic systems:

- **Natural Language Understanding**: Interpret commands expressed in everyday language
- **Contextual Reasoning**: Understand commands in the context of current robot state and environment
- **Task Decomposition**: Break complex tasks into sequences of simpler actions
- **Knowledge Integration**: Apply learned knowledge to novel situations
- **Adaptive Response**: Adjust responses based on feedback and environmental changes

### LLM Architecture for Robotics

The typical architecture for integrating LLMs with robotics includes:

1. **Input Processing**: Convert natural language to a format suitable for the LLM
2. **LLM Inference**: Process the input using the language model
3. **Output Parsing**: Extract actionable information from LLM responses
4. **ROS 2 Translation**: Convert to robot commands
5. **Execution Monitoring**: Track task progress and handle deviations

## Natural Language Processing for Robot Commands

### Command Interpretation Framework

Natural language commands to robots require a specialized interpretation framework that can handle the unique requirements of robotic systems:

- **Action Recognition**: Identify the specific actions requested by the user
- **Parameter Extraction**: Extract quantitative and qualitative parameters
- **Constraint Identification**: Recognize safety and operational constraints
- **Context Integration**: Use environmental and state information to refine interpretation

### Handling Ambiguity in Natural Language

Natural language often contains ambiguity that must be resolved for safe robot operation:

- **Reference Resolution**: Determine what objects or locations are being referenced
- **Quantifier Interpretation**: Convert vague quantities ("a bit", "a lot") to precise values
- **Temporal Reasoning**: Understand timing and sequence requirements
- **Spatial Reasoning**: Interpret spatial relationships and directions

### Example Command Processing

Consider the command: "Go to the kitchen and bring me the red cup from the table."

The processing pipeline would:
1. **Identify main task**: Fetch an object
2. **Decompose into subtasks**: Navigate to kitchen → locate red cup → grasp cup → return
3. **Extract parameters**: Target location (kitchen), object (red cup), source (table)
4. **Generate action sequence**: Navigation → perception → manipulation → navigation

## Task Decomposition Algorithms

### Hierarchical Task Networks (HTN)

HTN planning decomposes complex tasks into hierarchies of subtasks:

```
Fetch Object
├── Navigate to Location
│   ├── Plan Path
│   ├── Execute Navigation
│   └── Verify Arrival
├── Locate Object
│   ├── Perception Task
│   ├── Object Recognition
│   └── Pose Estimation
└── Manipulate Object
    ├── Approach Object
    ├── Grasp Object
    └── Verify Grasp
```

### Behavior Trees for Task Execution

Behavior trees provide a structured approach to organizing and executing decomposed tasks:

- **Selector Nodes**: Try different approaches until one succeeds
- **Sequence Nodes**: Execute steps in order until one fails
- **Decorator Nodes**: Modify behavior of child nodes
- **Action Nodes**: Execute specific robot actions

### Planning Logic and Sequence Generation

The planning system must generate executable sequences that account for:

- **Preconditions**: Conditions that must be true before executing actions
- **Effects**: Changes to the world state caused by actions
- **Dependencies**: Ordering constraints between actions
- **Resource Constraints**: Limited robot capabilities and resources

## ROS 2 Action Sequence Creation

### Understanding ROS 2 Actions

ROS 2 actions provide a goal-result-feedback communication pattern ideal for complex, multi-step tasks. Unlike services, actions can run for extended periods and provide progress updates.

### Action Architecture for Cognitive Planning

The cognitive planning system interfaces with ROS 2 actions through:

1. **Goal Generation**: Create action goals from LLM outputs
2. **Progress Monitoring**: Track action execution and handle failures
3. **Feedback Processing**: Use action feedback to update LLM context
4. **Result Interpretation**: Process action results to determine next steps

### Example Action Sequence

For a complex task like "Clean the living room," the system might generate:

```
1. [Navigation Action] Go to living room
2. [Perception Action] Scan room for objects
3. [Manipulation Action] Pick up trash items
4. [Navigation Action] Go to trash bin
5. [Manipulation Action] Dispose of trash
6. [Navigation Action] Return to living room
7. [Perception Action] Verify cleanliness
```

## Cognitive Reasoning in Robotics

### Situational Awareness

Cognitive planning systems must maintain awareness of:
- Robot state (location, battery, available resources)
- Environment state (object positions, obstacles, dynamic elements)
- Task progress (completed steps, pending actions)
- User context (preferences, previous interactions)

### Adaptive Planning

Robots must adapt their plans when:
- Environmental conditions change
- Initial assumptions prove incorrect
- Obstacles block planned paths
- Resources become unavailable

### Safety Integration

Cognitive planning must incorporate safety constraints:
- Physical safety limits
- Operational safety boundaries
- Ethical considerations
- Emergency response protocols

## Practical Implementation Example

Here's a comprehensive example of an LLM-based cognitive planning system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import openai
import json
import re

class LLMBotPlanner(Node):
    def __init__(self):
        super().__init__('llm_bot_planner')

        # Publishers for different robot capabilities
        self.nav_publisher = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.cmd_publisher = self.create_publisher(String, '/robot/command', 10)

        # Subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10)

        # Initialize LLM client (using OpenAI API as example)
        # In practice, you might use local models or other APIs
        self.llm_client = None  # Placeholder for actual LLM client

        self.robot_state = {
            'location': 'unknown',
            'battery': 100,
            'gripper': 'open',
            'current_task': None
        }

    def command_callback(self, msg):
        """Process natural language command using LLM"""
        natural_language = msg.data
        self.get_logger().info(f"Received command: {natural_language}")

        # Generate plan using LLM
        plan = self.generate_plan(natural_language)

        if plan:
            self.execute_plan(plan)
        else:
            self.get_logger().error("Failed to generate plan for command")

    def generate_plan(self, command):
        """Use LLM to generate a robot action plan from natural language"""
        # Define the system prompt for the LLM
        system_prompt = f"""
        You are a robot task planner. Convert natural language commands into structured robot action plans.
        Current robot state: {self.robot_state}

        Available actions:
        - NAVIGATE(location): Move robot to specified location
        - GRASP(object): Pick up specified object
        - PLACE(location): Place held object at location
        - SCAN(area): Look for objects in specified area
        - FOLLOW(path): Follow a specific path
        - WAIT(duration): Wait for specified duration

        Return a JSON array of actions with parameters. Each action should be a dictionary with 'action' and 'params' keys.
        Example: [{{"action": "NAVIGATE", "params": {{"location": "kitchen"}}}}, {{"action": "GRASP", "params": {{"object": "cup"}}}}]
        """

        try:
            # This is a simplified example - in practice you'd use an actual LLM API
            # For demonstration, we'll create a simple rule-based parser that mimics LLM behavior

            plan = self.parse_command_to_plan(command)
            self.get_logger().info(f"Generated plan: {plan}")
            return plan

        except Exception as e:
            self.get_logger().error(f"Error generating plan: {e}")
            return None

    def parse_command_to_plan(self, command):
        """Simplified command parser (in practice, use actual LLM)"""
        command = command.lower()

        if "go to" in command:
            # Extract location
            location_match = re.search(r"go to (\w+)", command)
            if location_match:
                location = location_match.group(1)
                return [{"action": "NAVIGATE", "params": {"location": location}}]

        elif "pick up" in command or "grasp" in command:
            # Extract object
            object_match = re.search(r"pick up (\w+)|grasp (\w+)", command)
            if object_match:
                obj = object_match.group(1) or object_match.group(2)
                return [{"action": "GRASP", "params": {"object": obj}}]

        elif "bring" in command and "to" in command:
            # Complex command: bring X to Y
            parts = command.split(" to ")
            if len(parts) == 2:
                bring_part = parts[0]  # "bring me the red cup"
                destination = parts[1]  # "the kitchen"

                # Extract object from bring part
                obj_match = re.search(r"the (\w+) (\w+)|the (\w+)", bring_part)
                if obj_match:
                    if obj_match.group(1) and obj_match.group(2):
                        obj = f"{obj_match.group(1)} {obj_match.group(2)}"
                    else:
                        obj = obj_match.group(3) or obj_match.group(1)

                    return [
                        {"action": "NAVIGATE", "params": {"location": "current_object_location"}},
                        {"action": "GRASP", "params": {"object": obj}},
                        {"action": "NAVIGATE", "params": {"location": destination}},
                        {"action": "PLACE", "params": {"location": destination}}
                    ]

        # Default: return simple navigation plan
        return [{"action": "NAVIGATE", "params": {"location": "default"}}]

    def execute_plan(self, plan):
        """Execute the generated action plan"""
        for action in plan:
            action_type = action['action']
            params = action['params']

            self.get_logger().info(f"Executing action: {action_type} with params: {params}")

            if action_type == 'NAVIGATE':
                self.execute_navigation(params['location'])
            elif action_type == 'GRASP':
                self.execute_grasp(params['object'])
            elif action_type == 'PLACE':
                self.execute_place(params['location'])
            elif action_type == 'SCAN':
                self.execute_scan(params['area'])
            elif action_type == 'FOLLOW':
                self.execute_follow(params['path'])
            elif action_type == 'WAIT':
                self.execute_wait(params['duration'])

    def execute_navigation(self, location):
        """Execute navigation to specified location"""
        # In a real implementation, this would send navigation goals to ROS 2 navigation stack
        goal_msg = PoseStamped()
        # Set appropriate pose based on location
        self.nav_publisher.publish(goal_msg.data)
        self.get_logger().info(f"Navigating to {location}")

    def execute_grasp(self, obj):
        """Execute grasping of specified object"""
        cmd_msg = String()
        cmd_msg.data = f"GRASP {obj}"
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f"Grasping {obj}")

    def execute_place(self, location):
        """Execute placing object at location"""
        cmd_msg = String()
        cmd_msg.data = f"PLACE {location}"
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f"Placing object at {location}")

    def execute_scan(self, area):
        """Execute scanning of specified area"""
        cmd_msg = String()
        cmd_msg.data = f"SCAN {area}"
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f"Scanning {area}")

    def execute_follow(self, path):
        """Execute following specified path"""
        cmd_msg = String()
        cmd_msg.data = f"FOLLOW {path}"
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f"Following path {path}")

    def execute_wait(self, duration):
        """Execute waiting for specified duration"""
        cmd_msg = String()
        cmd_msg.data = f"WAIT {duration}"
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f"Waiting for {duration} seconds")

def main(args=None):
    rclpy.init(args=args)
    planner = LLMBotPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates:
- Natural language command processing
- LLM-based plan generation (simulated)
- Task decomposition into action sequences
- ROS 2 integration for action execution
- State management and context awareness

## Performance Optimization for LLM Planning

### Model Selection and Optimization

- **Local vs Cloud Models**: Balance performance and privacy requirements
- **Model Quantization**: Reduce model size for faster inference
- **Caching Strategies**: Cache frequently used plan patterns
- **Batch Processing**: Process multiple commands efficiently

### Planning Efficiency

- **Hierarchical Planning**: Use high-level plans with detailed sub-plans
- **Plan Reuse**: Adapt existing plans for similar tasks
- **Parallel Execution**: Execute independent actions in parallel when safe
- **Predictive Planning**: Anticipate likely next actions

## Student Exercises

### Exercise 1: Basic LLM Integration
Implement a simple LLM-based planner that converts natural language commands to basic robot actions.

**Requirements:**
- Integrate with an LLM API or local model
- Handle basic navigation commands
- Generate simple action sequences
- Include error handling for invalid commands

### Exercise 2: Task Decomposition System
Create a more sophisticated system that decomposes complex tasks into multi-step action sequences.

**Requirements:**
- Handle multi-step commands like "Go to kitchen, pick up cup, bring to living room"
- Maintain state between actions
- Include feedback mechanisms
- Validate plan feasibility before execution

## Troubleshooting and Best Practices

### Common Issues

- **Overly Complex Plans**: LLMs may generate plans that are too complex for the robot
- **Context Loss**: LLMs may lose track of robot state between interactions
- **Safety Violations**: Generated plans may not consider safety constraints
- **Performance Bottlenecks**: LLM inference may be too slow for real-time applications

### Best Practices

- **Plan Validation**: Always validate LLM-generated plans before execution
- **Safety Constraints**: Implement hard safety limits that override LLM decisions
- **Fallback Mechanisms**: Provide manual control when LLM plans fail
- **Continuous Learning**: Use execution feedback to improve planning over time

## Summary

This chapter covered the cognitive planning layer of VLA systems:

- LLM integration with robotics applications
- Natural language processing for robot commands
- Task decomposition algorithms and planning logic
- ROS 2 action sequence creation
- Cognitive reasoning and adaptive planning

The cognitive planning system serves as the intelligent bridge between human intent and robot action, enabling sophisticated autonomous behaviors based on natural language input.