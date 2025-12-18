---
sidebar_position: 3
---

# Chapter 3: Path Planning with Nav2

## Introduction to Navigation2 Framework

Navigation2 (Nav2) is the ROS 2 navigation framework that provides a complete solution for autonomous robot navigation. It builds upon the lessons learned from the ROS 1 navigation stack while incorporating modern software engineering practices and new capabilities for autonomous navigation.

Nav2 is designed to be modular, extensible, and robust, providing a comprehensive set of tools for path planning, execution, and recovery in complex environments. The framework is particularly well-suited for humanoid robots when properly configured for bipedal locomotion.

## Nav2 Architecture and Components

The Navigation2 framework is built around a behavior tree architecture that orchestrates different navigation capabilities:

### Core Components

1. **Navigator**: High-level navigation orchestrator
2. **Planner Server**: Global and local path planning
3. **Controller Server**: Local trajectory control
4. **Recovery Server**: Behavior-based recovery
5. **BT Navigator**: Behavior tree-based navigation executor

### Behavior Tree Architecture

Nav2 uses behavior trees to define navigation behaviors:

```
NavigateToPose
├── ComputePathToPose (Global Planner)
├── FollowPath (Local Controller)
│   ├── SmoothPath
│   ├── ComputeVelocityCommands
│   ├── IsGoalReached
│   └── IsPathValid
└── RecoveryNode
    ├── Spin
    ├── Backup
    └── Wait
```

## Bipedal Humanoid Path Planning Considerations

Bipedal humanoid robots present unique challenges for path planning that differ significantly from wheeled or tracked robots:

### Kinematic Constraints

- **Foot placement**: Path must account for discrete footstep planning
- **Balance maintenance**: Continuous balance during locomotion
- **Turning radius**: Limited by leg span and balance constraints
- **Step height**: Maximum obstacle clearance capability

### Dynamic Constraints

- **Center of Mass (CoM)**: Must remain within support polygon
- **Zero Moment Point (ZMP)**: Stability criterion for bipedal gait
- **Walking speed**: Limited by balance and step frequency
- **Energy efficiency**: Optimized for battery-powered operation

### Terrain Considerations

- **Step-overs**: Ability to step over small obstacles
- **Slope limitations**: Maximum incline/decline angles
- **Surface types**: Different coefficients of friction
- **Stair navigation**: Specialized gait patterns required

## Navigation in Complex Environments

Nav2 provides sophisticated capabilities for navigating complex environments, which is particularly important for humanoid robots that need to operate in human spaces:

### Global Path Planning

- **A* and Dijkstra algorithms**: Optimal path computation
- **Grid-based planners**: Efficient for 2D navigation
- **Sampling-based planners**: For high-dimensional spaces
- **Topological planners**: For large-scale navigation

### Local Path Planning

- **Dynamic Window Approach (DWA)**: Real-time obstacle avoidance
- **Timed Elastic Band (TEB)**: Time-optimal trajectory planning
- **MPC-based controllers**: Model Predictive Control for complex dynamics

### Costmap Management

- **Static layer**: Fixed obstacles from map
- **Obstacle layer**: Dynamic obstacles from sensors
- **Inflation layer**: Safety margins around obstacles
- **Voxel layer**: 3D obstacle representation

## Nav2 Configuration for Humanoid Robots

Configuring Nav2 for bipedal humanoid robots requires special attention to the robot's unique characteristics:

### Example Configuration File

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the behavior tree files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICController"
      debug: false
      # Humanoid-specific parameters
      max_linear_speed: 0.3      # Slower for balance
      min_linear_speed: 0.05
      max_angular_speed: 0.5
      min_angular_speed: 0.1
      speed_scaling_radius: 0.5
      inflation_cost_scaling_factor: 3.0  # Higher for humanoid safety

progress_checker:
  ros__parameters:
    use_sim_time: True
    plugin: "nav2_controller::SimpleProgressChecker"
    required_movement_radius: 0.5  # Humanoid-specific
    movement_time_allowance: 10.0

goal_checker:
  ros__parameters:
    use_sim_time: True
    plugin: "nav2_controller::SimpleGoalChecker"
    xy_goal_tolerance: 0.3    # Larger for humanoid
    yaw_goal_tolerance: 0.2
    stateful: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_footprint"
      use_sim_time: True
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
      # Humanoid-specific inflation
      inflation_radius: 0.8    # Larger for humanoid safety margin
      cost_scaling_factor: 3.0
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        voxel_size: 0.5
        observation_sources: scan
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0  # Humanoid height consideration
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.8

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_footprint"
      use_sim_time: True
      robot_radius: 0.4  # Humanoid-specific
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/laser_scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 1.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5    # Humanoid-specific
      use_astar: false
      allow_unknown: true
```

### Humanoid-Specific Parameters

- **Robot radius**: Increased for humanoid safety margin
- **Goal tolerances**: Adjusted for bipedal locomotion precision
- **Speed limits**: Reduced for stability during walking
- **Inflation radius**: Larger safety margins for bipedal robots
- **Step height**: Consideration for obstacle clearance

## Behavior Trees and Navigation Behaviors

Nav2 uses behavior trees to define complex navigation behaviors that are particularly important for humanoid robots:

### Custom Behavior Trees for Humanoid Navigation

```xml
<!-- humanoid_navigate_to_pose_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="6" name="NavigateRecovery">
          <PipelineSequence name="NavigateWithSmoothing">
            <RecoveryNode number_of_retries="2" name="ComputeAndSmoothPathRecovery">
              <PipelineSequence name="ComputeAndSmooth">
                <RecoveryNode number_of_retries="2" name="ComputePathRecovery">
                  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                  <ReactiveStop name="StopAtGoal" path="{path}"/>
                </RecoveryNode>
                <SmoothPath path="{path}" output_path="{smoothed_path}" smoother_id="SimpleSmoother"/>
              </PipelineSequence>
              <BackUp backup_dist="0.15" backup_speed="0.05" name="BackUpRobot"/>
            </RecoveryNode>
            <FollowPath path="{smoothed_path}" controller_id="FollowPath"/>
          </RecoveryNode>
        </RecoveryNode>
      </RateController>
      <ReactiveStop name="FinalApproachStop" path="{smoothed_path}"/>
    </PipelineSequence>
  </BehaviorTree>

  <BehaviorTree ID="RecoveryNode">
    <ReactiveSequence>
      <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
      <RoundRobin name="RecoveryActions">
        <Spin spin_dist="1.57" name="Spin"/>
        <BackUp backup_dist="0.15" backup_speed="0.05" name="BackUp"/>
        <Wait wait_duration="5" name="Wait"/>
      </RoundRobin>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

## Practical Examples with Code Snippets

### Example 1: Humanoid Path Planning Node

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Create action client for Nav2
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Humanoid-specific parameters
        self.step_size = 0.3  # Maximum step size for humanoid
        self.balance_margin = 0.5  # Safety margin for balance
        self.max_slope = 15.0  # Maximum incline in degrees

    def navigate_to_pose(self, x, y, theta):
        """Send navigation goal to Nav2 with humanoid constraints"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Wait for action server
        self.nav_to_pose_client.wait_for_server()

        # Send goal
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current pose: {feedback.current_pose}')

    def validate_path_for_humanoid(self, path):
        """Validate path for humanoid-specific constraints"""
        # Check for excessive slopes
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position

            # Calculate distance between consecutive points
            dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

            # Check if step size is within humanoid limits
            if dist > self.step_size:
                self.get_logger().warn(f'Step size {dist} exceeds humanoid limit {self.step_size}')
                return False

        return True
```

### Example 2: Custom Footstep Planner for Humanoid

```python
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Header

class FootstepPlanner:
    def __init__(self):
        self.step_length = 0.3  # Distance between consecutive steps
        self.step_width = 0.2   # Lateral distance between feet
        self.max_step_height = 0.1  # Maximum step-over height

    def plan_footsteps(self, path, start_pose):
        """Plan footstep sequence from navigation path"""
        footsteps = []

        # Initialize with starting position
        left_foot = self.calculate_initial_foot_position(start_pose, 'left')
        right_foot = self.calculate_initial_foot_position(start_pose, 'right')

        footsteps.append(('left', left_foot))
        footsteps.append(('right', right_foot))

        # Plan footsteps along the path
        current_pose = start_pose
        path_index = 0

        while path_index < len(path.poses) - 1:
            # Calculate next step position based on path
            next_pose = path.poses[path_index]

            # Determine which foot to move based on gait pattern
            if len(footsteps) % 2 == 0:  # Even steps - move right foot
                next_foot_pos = self.calculate_next_foot_position(
                    current_pose, next_pose, 'right')
                footsteps.append(('right', next_foot_pos))
            else:  # Odd steps - move left foot
                next_foot_pos = self.calculate_next_foot_position(
                    current_pose, next_pose, 'left')
                footsteps.append(('left', next_foot_pos))

            current_pose = next_pose
            path_index += 1

        return footsteps

    def calculate_initial_foot_position(self, pose, foot_type):
        """Calculate initial foot position based on robot pose"""
        # For initial position, feet are placed side by side
        pos = Point()
        pos.x = pose.pose.position.x
        pos.y = pose.pose.position.y
        pos.z = pose.pose.position.z

        # Offset for left or right foot
        if foot_type == 'left':
            pos.y += self.step_width / 2
        else:  # right
            pos.y -= self.step_width / 2

        return pos

    def calculate_next_foot_position(self, current_pose, target_pose, foot_type):
        """Calculate next foot position"""
        # Calculate direction vector
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        dist = np.sqrt(dx**2 + dy**2)

        # Normalize and scale to step length
        if dist > 0:
            dx = dx * self.step_length / dist
            dy = dy * self.step_length / dist

        next_pos = Point()
        next_pos.x = current_pose.pose.position.x + dx
        next_pos.y = current_pose.pose.position.y + dy
        next_pos.z = current_pose.pose.position.z  # Assuming flat terrain

        return next_pos

    def validate_footstep_for_terrain(self, foot_pos, terrain_map):
        """Validate if footstep is safe for current terrain"""
        # Check if footstep is on stable ground
        # Check for obstacles at foot level
        # Check for sufficient clearance

        # This would involve checking terrain height, slope, and obstacles
        return True  # Simplified for example
```

## Integration with VSLAM Localization System

For humanoid robots, proper integration between path planning and localization is crucial:

### Localization Feedback Loop

```python
class HumanoidNavigationManager(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_manager')

        # Subscriptions for localization
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # VSLAM pose subscription
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.vslam_pose_callback,
            10
        )

        # Publishers for navigation
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Initialize components
        self.current_pose = None
        self.vslam_pose = None
        self.localization_fusion = LocalizationFusion()

    def pose_callback(self, msg):
        """Handle AMCL pose updates"""
        self.current_pose = msg.pose.pose

    def vslam_pose_callback(self, msg):
        """Handle VSLAM pose updates"""
        self.vslam_pose = msg.pose

    def fuse_localization_data(self):
        """Fuse AMCL and VSLAM localization data"""
        if self.current_pose and self.vslam_pose:
            # Combine both localization sources
            fused_pose = self.localization_fusion.fuse_poses(
                self.current_pose,
                self.vslam_pose
            )
            return fused_pose
        return self.current_pose or self.vslam_pose
```

## Student Exercises for Hands-on Practice

### Exercise 1: Nav2 Configuration for Humanoid Robot
Configure Nav2 for a simulated humanoid robot and test path planning in simple environments. Adjust parameters for humanoid-specific constraints.

### Exercise 2: Behavior Tree Customization
Create a custom behavior tree for humanoid navigation that includes specialized recovery behaviors for bipedal locomotion challenges.

### Exercise 3: Path Validation for Humanoid Constraints
Implement a path validation system that checks navigation paths for humanoid-specific constraints like step size, slope limits, and balance margins.

## Summary

This chapter covered Path Planning with Nav2 for humanoid robots, including the Navigation2 framework architecture, bipedal humanoid path planning considerations, navigation in complex environments, and Nav2 configuration for humanoid robots.

We explored the unique challenges that bipedal humanoid robots present for path planning, including kinematic and dynamic constraints, and how Nav2 can be configured to address these challenges. The chapter provided practical examples of humanoid-specific path planning implementations and demonstrated integration with VSLAM localization systems.

The behavior tree architecture of Nav2 allows for sophisticated navigation behaviors that are essential for humanoid robots operating in human environments. Proper configuration of safety margins, speed limits, and recovery behaviors is crucial for successful humanoid navigation.

With this chapter, we have completed the Isaac AI Robot Brain module, covering Isaac Sim for synthetic data generation, Isaac ROS for hardware-accelerated VSLAM, and Nav2 for humanoid path planning. These technologies form a complete autonomy pipeline for humanoid robots.