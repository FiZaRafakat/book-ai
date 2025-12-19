# VLA API Contract: Vision-Language-Action Interfaces

## Overview
This document specifies the conceptual interfaces and contracts that students will learn to implement in the VLA Robotics Module. These represent the key APIs and interfaces used in VLA systems.

## Voice Recognition Interface

### VoiceRecognitionService

```yaml
name: VoiceRecognitionService
description: Converts speech input to text commands
methods:
  - name: recognizeSpeech
    input:
      type: AudioStream
      properties:
        audio_data: binary
        sample_rate: integer
        language: string
    output:
      type: RecognitionResult
      properties:
        text: string
        confidence: number (0.0-1.0)
        timestamp: string (ISO 8601)
    errors:
      - AudioTooNoisy
      - RecognitionFailed
      - UnsupportedLanguage
```

## LLM Planning Interface

### CognitivePlannerService

```yaml
name: CognitivePlannerService
description: Translates natural language to robot action sequences
methods:
  - name: planTask
    input:
      type: NaturalLanguageRequest
      properties:
        command: string
        context: object
        robot_capabilities: array
    output:
      type: ActionSequence
      properties:
        actions: array
        parameters: object
        estimated_duration: number (seconds)
    errors:
      - InvalidCommand
      - TaskTooComplex
      - CapabilityNotAvailable
```

## Robot Action Interface

### RobotActionExecutor

```yaml
name: RobotActionExecutor
description: Executes action sequences on the robot
methods:
  - name: executeActionSequence
    input:
      type: ActionSequence
      properties:
        actions: array
        parameters: object
        timeout: number
    output:
      type: ExecutionResult
      properties:
        status: string (pending, executing, completed, failed)
        progress: number (0.0-1.0)
        completed_actions: array
    errors:
      - ExecutionFailed
      - SafetyViolation
      - RobotBusy
```

## ROS 2 Message Definitions

### VoiceCommand.msg
```text
string text
float32 confidence
string[] context_keywords
builtin_interfaces/Time timestamp
```

### ActionStep.msg
```text
string action_type  # "NAVIGATE", "GRASP", "PLACE", etc.
string parameters_json
builtin_interfaces/Time estimated_duration
bool safety_critical
```

### ActionSequence.msg
```text
ActionStep[] steps
string sequence_id
builtin_interfaces/Time created_timestamp
string[] dependencies
bool interruptible
```

## ROS 2 Service Definitions

### PlanAction.srv
```text
# Request
string natural_language_command
string[] environmental_context
string[] robot_capabilities

---
# Response
ActionSequence sequence
bool planning_successful
string error_message
float32 confidence
```

### ExecuteSequence.srv
```text
# Request
ActionSequence sequence
float32 timeout_seconds
bool allow_interruption

---
# Response
bool execution_successful
string[] completed_actions
string[] failed_actions
string error_message
```

## Simulation Interface

### SimulationEnvironmentInterface

```yaml
name: SimulationEnvironmentInterface
description: Provides simulation environment information
methods:
  - name: getEnvironmentState
    input: {}
    output:
      type: EnvironmentState
      properties:
        object_positions: array
        robot_position: object
        navigation_map: object
        sensor_readings: object
    errors:
      - SimulationNotReady
      - EnvironmentStateUnavailable
```

## Error Handling Contract

### Standard Error Format
```json
{
  "error_code": "string",
  "message": "string",
  "details": "object",
  "timestamp": "ISO 8601 string",
  "retriable": "boolean"
}
```

### Common Error Codes
- `VOICE_RECOGNITION_FAILED`: Speech recognition could not process input
- `COMMAND_AMBIGUOUS`: Natural language command was unclear
- `TASK_NOT_PLANNABLE`: LLM could not generate valid action sequence
- `ROBOT_BUSY`: Robot is currently executing another task
- `SAFETY_VIOLATION`: Action would violate safety constraints
- `ENVIRONMENT_UNCHANGED`: Expected environmental change did not occur
- `SIMULATION_ERROR`: Simulation environment error occurred

## Performance Requirements

### Response Time SLAs
- Voice recognition: < 1 second for 5-second audio clip
- LLM planning: < 3 seconds for simple commands, < 10 seconds for complex tasks
- Action execution: < 100ms for simple actions, variable for complex sequences
- System startup: < 30 seconds for full VLA pipeline

### Reliability Requirements
- Voice recognition accuracy: > 85% in quiet environments
- Task completion rate: > 90% for valid commands
- System availability: > 99% during simulation runs
- Error recovery: < 5 seconds to resume normal operation after error

## Safety Constraints

### Hard Safety Limits
- Maximum navigation speed: 1.0 m/s
- Maximum manipulation force: 50N
- Minimum obstacle distance: 0.1m
- Maximum operation time: 30 minutes without human intervention

### Safety Checkpoints
- Validate all action sequences before execution
- Monitor robot state during execution
- Implement emergency stop capability
- Log all safety-related events