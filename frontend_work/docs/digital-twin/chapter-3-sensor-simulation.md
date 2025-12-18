---
sidebar_position: 3
---

# Chapter 3: Sensor Simulation

## Introduction to Sensor Simulation in Digital Twins

Sensor simulation is a critical component of digital twin systems for humanoid robots, as it provides the virtual sensory input that enables AI systems to perceive and interact with the simulated environment. In a digital twin, sensors bridge the gap between the physical world simulation (handled by physics engines like those in Gazebo) and the visual representation (handled by rendering engines like Unity), creating a complete perceptual experience for AI algorithms.

The primary goal of sensor simulation is to generate realistic sensor data that closely matches what would be produced by actual hardware sensors on a physical robot. This includes not only the ideal sensor readings but also realistic noise, delays, and imperfections that characterize real-world sensors. By accurately simulating these characteristics, digital twins can be used to train AI models that will perform effectively when deployed on real robots.

## Sensor Plugin Architecture in Gazebo and Unity

Both Gazebo and Unity support plugin architectures that allow for the implementation of various sensor types. Understanding these architectures is essential for creating realistic sensor simulations.

### Gazebo Sensor Plugins

In Gazebo, sensors are implemented as plugins that interface with the physics engine to generate realistic sensor data. The plugin architecture allows for:

- **Physics-based sensing**: Sensors interact with the physics simulation to generate realistic data
- **ROS 2 integration**: Direct publishing of sensor data to ROS 2 topics
- **Custom sensor types**: Development of specialized sensors beyond standard types
- **Noise modeling**: Built-in support for adding realistic sensor noise and imperfections

### Unity Sensor Simulation

Unity's approach to sensor simulation often involves:

- **Rendering-based sensors**: Using Unity's rendering pipeline to generate visual sensor data
- **Raycasting for depth sensors**: Using Unity's physics engine for LiDAR and depth sensing
- **Integration with ROS#**: Publishing sensor data to ROS 2 through Unity-ROS bridges
- **Custom shader implementations**: Specialized shaders for generating sensor-specific data

## LiDAR Simulation: Ray Tracing and Noise Modeling

LiDAR (Light Detection and Ranging) sensors are crucial for humanoid robots, providing 3D spatial information about the environment. Simulating LiDAR sensors requires accurate ray tracing and realistic noise modeling.

### LiDAR Implementation in Gazebo

```xml
<sensor name="lidar_sensor" type="ray">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
        <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

### Adding Noise to LiDAR Data

Real LiDAR sensors have various sources of noise and uncertainty. In simulation, this can be modeled as:

```python
import numpy as np
from sensor_msgs.msg import LaserScan

def add_lidar_noise(scan_msg, noise_std=0.02):
    """
    Add realistic noise to LiDAR scan data
    """
    # Convert to numpy array for processing
    ranges = np.array(scan_msg.ranges)

    # Remove invalid ranges for noise calculation
    valid_mask = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)

    # Add Gaussian noise to valid ranges
    noise = np.random.normal(0, noise_std, len(ranges))
    noise[~valid_mask] = 0  # Don't add noise to invalid ranges

    # Apply noise
    ranges_with_noise = ranges + noise

    # Ensure ranges stay within valid bounds
    ranges_with_noise = np.clip(ranges_with_noise,
                               scan_msg.range_min,
                               scan_msg.range_max)

    # Update the message
    scan_msg.ranges = ranges_with_noise.tolist()

    return scan_msg

def simulate_lidar_systematic_errors(scan_msg):
    """
    Simulate systematic errors in LiDAR sensors
    """
    ranges = np.array(scan_msg.ranges)

    # Add distance-dependent bias (common in real LiDAR)
    distance_bias = 0.001 * ranges  # 1mm per meter

    # Add angle-dependent bias
    angle_indices = np.arange(len(ranges))
    angle_bias = 0.005 * np.sin(2 * np.pi * angle_indices / len(ranges))

    # Apply systematic errors
    ranges_with_systematic_error = ranges + distance_bias + angle_bias

    # Ensure ranges stay within valid bounds
    ranges_with_systematic_error = np.clip(ranges_with_systematic_error,
                                          scan_msg.range_min,
                                          scan_msg.range_max)

    scan_msg.ranges = ranges_with_systematic_error.tolist()
    return scan_msg
```

## Depth Camera Simulation: Point Clouds and RGB-D Data

Depth cameras provide both color (RGB) and depth (D) information, making them valuable for humanoid robots that need both visual recognition and spatial understanding.

### Depth Camera Configuration in Gazebo

```xml
<sensor name="depth_camera" type="depth">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>depth_camera</cameraName>
    <imageTopicName>/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>depth_camera_frame</frameName>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### Point Cloud Generation and Processing

```python
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def depth_image_to_pointcloud(depth_image, camera_info):
    """
    Convert depth image to point cloud
    """
    # Extract camera parameters
    fx = camera_info.K[0]  # Focal length x
    fy = camera_info.K[4]  # Focal length y
    cx = camera_info.K[2]  # Principal point x
    cy = camera_info.K[5]  # Principal point y

    height, width = depth_image.shape

    # Create coordinate grids
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # Convert pixel coordinates to camera coordinates
    z = depth_image  # Depth values
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Flatten arrays and remove invalid points
    valid_mask = (z > 0) & (z < np.inf)
    points = np.column_stack([x[valid_mask], y[valid_mask], z[valid_mask]])

    return points

def add_depth_camera_noise(points, noise_std=0.001):
    """
    Add realistic noise to point cloud data
    """
    # Add Gaussian noise to each coordinate
    noise = np.random.normal(0, noise_std, points.shape)
    noisy_points = points + noise

    return noisy_points

def create_pointcloud2_message(points, frame_id="depth_camera_frame"):
    """
    Create a PointCloud2 message from numpy array
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Define point fields
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Create PointCloud2 message
    pointcloud_msg = pc2.create_cloud(header, fields, points)

    return pointcloud_msg
```

## IMU Simulation: Acceleration, Angular Velocity, and Drift Modeling

Inertial Measurement Units (IMUs) provide crucial information about robot orientation, acceleration, and angular velocity. Accurate IMU simulation is essential for humanoid robots that need to maintain balance and understand their motion state.

### IMU Configuration in Gazebo

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
    <topicName>imu</topicName>
    <bodyName>imu_link</bodyName>
    <updateRateHZ>100.0</updateRateHZ>
    <gaussianNoise>0.0</gaussianNoise>
    <frameName>imu_link</frameName>
    <initialOrientationAsReference>false</initialOrientationAsReference>
  </plugin>
</sensor>
```

### IMU Drift and Bias Simulation

```python
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import rospy

class ImuSimulator:
    def __init__(self):
        # IMU noise parameters (typical for MEMS IMUs)
        self.gyro_noise_density = 0.00015  # rad/s/sqrt(Hz)
        self.accel_noise_density = 0.005   # m/s^2/sqrt(Hz)

        # Bias parameters
        self.gyro_bias_drift = 0.0001     # rad/s/sqrt(s)
        self.accel_bias_drift = 0.001     # m/s^2/sqrt(s)

        # Current bias values (random walk)
        self.current_gyro_bias = np.random.normal(0, 0.01, 3)
        self.current_accel_bias = np.random.normal(0, 0.05, 3)

        # Timing
        self.last_update_time = rospy.Time.now()

    def simulate_imu_data(self, true_angular_velocity, true_linear_acceleration, dt):
        """
        Simulate realistic IMU data with noise, bias, and drift
        """
        # Update biases with random walk
        self.current_gyro_bias += np.random.normal(0, self.gyro_bias_drift * np.sqrt(dt), 3)
        self.current_accel_bias += np.random.normal(0, self.accel_bias_drift * np.sqrt(dt), 3)

        # Calculate noise based on sample time
        gyro_noise = np.random.normal(0, self.gyro_noise_density / np.sqrt(dt), 3)
        accel_noise = np.random.normal(0, self.accel_noise_density / np.sqrt(dt), 3)

        # Apply noise, bias, and drift to true values
        measured_angular_velocity = true_angular_velocity + self.current_gyro_bias + gyro_noise
        measured_linear_acceleration = true_linear_acceleration + self.current_accel_bias + accel_noise

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.angular_velocity = Vector3(
            x=measured_angular_velocity[0],
            y=measured_angular_velocity[1],
            z=measured_angular_velocity[2]
        )

        imu_msg.linear_acceleration = Vector3(
            x=measured_linear_acceleration[0],
            y=measured_linear_acceleration[1],
            z=measured_linear_acceleration[2]
        )

        # For quaternion, we'll use a simple integration approach
        # In a real implementation, you'd integrate the angular velocity
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Set covariance matrices (information about measurement uncertainty)
        imu_msg.angular_velocity_covariance = [
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001
        ]

        imu_msg.linear_acceleration_covariance = [
            0.0025, 0, 0,
            0, 0.0025, 0,
            0, 0, 0.0025
        ]

        return imu_msg

    def reset_bias(self):
        """Reset IMU biases to initial values"""
        self.current_gyro_bias = np.random.normal(0, 0.01, 3)
        self.current_accel_bias = np.random.normal(0, 0.05, 3)
```

## Camera Sensors: Stereo Vision, Wide-Angle, and Fisheye Simulation

Camera sensors provide visual information that is crucial for humanoid robots to recognize objects, navigate environments, and interact with humans. Different types of camera sensors serve different purposes.

### Standard Camera Configuration

```xml
<sensor name="camera" type="camera">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <camera name="head_camera">
    <horizontal_fov>1.396</horizontal_fov>  <!-- 80 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>0.0</updateRate>
    <cameraName>camera</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>camera_frame</frameName>
    <hackBaseline>0.07</hackBaseline>
    <distortionK1>0.0</distortionK1>
    <distortionK2>0.0</distortionK2>
    <distortionK3>0.0</distortionK3>
    <distortionT1>0.0</distortionT1>
    <distortionT2>0.0</distortionT2>
  </plugin>
</sensor>
```

### Stereo Camera Configuration

```xml
<!-- Left camera -->
<sensor name="stereo_left" type="camera">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <camera name="left_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="left_camera_controller" filename="libgazebo_ros_camera.so">
    <cameraName>stereo/left</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>stereo_left_frame</frameName>
  </plugin>
</sensor>

<!-- Right camera -->
<sensor name="stereo_right" type="camera">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <camera name="right_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="right_camera_controller" filename="libgazebo_ros_camera.so">
    <cameraName>stereo/right</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>stereo_right_frame</frameName>
  </plugin>
</sensor>
```

## Force/Torque Sensor Simulation

Force/torque sensors are essential for humanoid robots to understand their interaction with the environment, particularly for manipulation and balance control.

### Force/Torque Sensor Configuration

```xml
<sensor name="ft_sensor" type="force_torque">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>sensor</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
  <plugin name="ft_controller" filename="libgazebo_ros_ft_sensor.so">
    <topicName>ft_sensor</topicName>
    <jointName>ee_joint</jointName>
  </plugin>
</sensor>
```

### Force/Torque Data Processing

```python
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Header
import rospy

class ForceTorqueSimulator:
    def __init__(self):
        # Noise parameters for force/torque sensor
        self.force_noise_std = 0.1    # Newtons
        self.torque_noise_std = 0.01  # Newton-meters

        # Bias parameters
        self.force_bias_drift = 0.001
        self.torque_bias_drift = 0.0001

        # Current bias values
        self.current_force_bias = np.random.normal(0, 0.1, 3)
        self.current_torque_bias = np.random.normal(0, 0.01, 3)

    def simulate_force_torque(self, true_force, true_torque, dt):
        """
        Simulate force/torque sensor data with noise and drift
        """
        # Update biases
        self.current_force_bias += np.random.normal(0, self.force_bias_drift * np.sqrt(dt), 3)
        self.current_torque_bias += np.random.normal(0, self.torque_bias_drift * np.sqrt(dt), 3)

        # Add noise
        force_noise = np.random.normal(0, self.force_noise_std, 3)
        torque_noise = np.random.normal(0, self.torque_noise_std, 3)

        # Apply noise and bias
        measured_force = true_force + self.current_force_bias + force_noise
        measured_torque = true_torque + self.current_torque_bias + torque_noise

        # Create wrench message
        wrench_msg = Wrench()
        wrench_msg.force = Vector3(
            x=measured_force[0],
            y=measured_force[1],
            z=measured_force[2]
        )
        wrench_msg.torque = Vector3(
            x=measured_torque[0],
            y=measured_torque[1],
            z=measured_torque[2]
        )

        return wrench_msg
```

## Sensor Fusion in Digital Twins

Sensor fusion combines data from multiple sensors to provide more accurate and robust perception than any single sensor could provide alone.

### Basic Sensor Fusion Example

```python
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion

class SensorFusion:
    def __init__(self):
        # Initialize state estimate (position, orientation, velocities)
        self.position = np.zeros(3)
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity rotation
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Covariance matrices for uncertainty tracking
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(3) * 0.1

    def update_with_imu(self, imu_msg, dt):
        """
        Update state estimate using IMU data
        """
        # Extract measurements
        angular_vel = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])

        linear_acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Integrate angular velocity to update orientation
        # Simple first-order integration (in practice, use more sophisticated methods)
        delta_angle = angular_vel * dt
        delta_rotation = R.from_rotvec(delta_angle)
        self.orientation = self.orientation * delta_rotation

        # Transform acceleration to world frame and integrate
        world_acc = self.orientation.apply(linear_acc)
        self.linear_velocity += world_acc * dt
        self.position += self.linear_velocity * dt

        # Update uncertainty based on IMU noise characteristics
        imu_cov = np.array(imu_msg.linear_acceleration_covariance).reshape(3, 3)
        self.position_covariance += imu_cov * dt * dt

    def update_with_camera(self, position_measurement, measurement_covariance):
        """
        Update state estimate using camera position measurement
        """
        # Simple Kalman filter update for position
        # In practice, would use more sophisticated fusion techniques

        # Innovation (difference between measurement and prediction)
        innovation = position_measurement - self.position

        # Innovation covariance
        S = self.position_covariance + measurement_covariance

        # Kalman gain
        K = self.position_covariance @ np.linalg.inv(S)

        # Update state
        self.position += K @ innovation

        # Update covariance
        self.position_covariance = (np.eye(3) - K) @ self.position_covariance

    def get_fused_pose(self):
        """
        Get the current fused pose estimate
        """
        pose = Pose()
        pose.position = Point(
            x=self.position[0],
            y=self.position[1],
            z=self.position[2]
        )

        quat = self.orientation.as_quat()
        pose.orientation = Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )

        return pose
```

## Noise Modeling Techniques

Realistic noise modeling is crucial for creating sensor simulations that effectively prepare AI systems for deployment on real hardware.

### Types of Sensor Noise

1. **Gaussian Noise**: Random noise following a normal distribution
2. **Bias**: Systematic offset in measurements
3. **Drift**: Slowly changing bias over time
4. **Quantization**: Discrete steps due to digital sampling
5. **Outliers**: Occasional large errors due to interference

### Comprehensive Noise Model

```python
import numpy as np

class SensorNoiseModel:
    def __init__(self, base_noise_std, bias_drift_rate, outlier_rate=0.01):
        self.base_noise_std = base_noise_std
        self.bias_drift_rate = bias_drift_rate
        self.outlier_rate = outlier_rate

        # Initialize bias with random walk
        self.current_bias = 0.0
        self.time_since_last_bias_update = 0.0

    def add_noise(self, true_value, dt):
        """
        Add comprehensive noise model to sensor measurement
        """
        # Update bias with random walk
        self.current_bias += np.random.normal(0, self.bias_drift_rate * np.sqrt(dt))

        # Add base Gaussian noise
        noise = np.random.normal(0, self.base_noise_std)

        # Add the noise to the true value
        noisy_value = true_value + self.current_bias + noise

        # Occasionally add outliers
        if np.random.random() < self.outlier_rate:
            outlier = np.random.normal(0, self.base_noise_std * 10)  # Much larger noise
            noisy_value += outlier

        return noisy_value

    def reset(self):
        """Reset the noise model"""
        self.current_bias = 0.0
        self.time_since_last_bias_update = 0.0

# Example usage for different sensor types
class MultiSensorNoiseModel:
    def __init__(self):
        # Different noise characteristics for different sensors
        self.lidar_noise = SensorNoiseModel(
            base_noise_std=0.02,      # 2cm
            bias_drift_rate=0.0001,   # Very slow drift
            outlier_rate=0.001        # Rare outliers
        )

        self.camera_noise = SensorNoiseModel(
            base_noise_std=0.5,       # 0.5 pixel
            bias_drift_rate=0.001,    # Moderate drift
            outlier_rate=0.005        # Some outliers possible
        )

        self.imu_noise = SensorNoiseModel(
            base_noise_std=0.001,     # Low noise for modern IMUs
            bias_drift_rate=0.01,     # Significant drift over time
            outlier_rate=0.002        # Occasional spikes
        )
```

## Sensor Calibration in Simulation

Sensor calibration in simulation helps ensure that the virtual sensors match the characteristics of their real-world counterparts.

### Calibration Parameters

```python
class SensorCalibration:
    def __init__(self):
        # Camera intrinsic parameters
        self.camera_matrix = np.array([
            [525.0, 0.0, 319.5],  # fx, 0, cx
            [0.0, 525.0, 239.5],  # 0, fy, cy
            [0.0, 0.0, 1.0]       # 0, 0, 1
        ])

        # Distortion coefficients [k1, k2, p1, p2, k3]
        self.distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # LiDAR calibration
        self.lidar_resolution = 0.01  # 1cm resolution
        self.lidar_accuracy = 0.02    # 2cm accuracy

        # IMU calibration
        self.accel_bias = np.array([0.01, -0.02, 0.05])  # m/s^2
        self.gyro_bias = np.array([0.001, -0.002, 0.003])  # rad/s
        self.accel_scale_factor = 1.005  # Slight scaling error
        self.gyro_scale_factor = 0.998   # Slight scaling error

    def apply_camera_calibration(self, points_2d):
        """
        Apply camera calibration to 2D points
        """
        # Apply distortion correction
        distorted_points = self._apply_distortion(points_2d)

        # Apply intrinsic matrix transformation
        undistorted_points = self._apply_intrinsic_matrix(distorted_points)

        return undistorted_points

    def _apply_distortion(self, points):
        """
        Apply lens distortion to points
        """
        x, y = points[:, 0], points[:, 1]

        k1, k2, p1, p2, k3 = self.distortion_coeffs

        # Radial distortion
        r2 = x*x + y*y
        radial_distortion = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2

        # Tangential distortion
        x_distorted = x * radial_distortion + 2*p1*x*y + p2*(r2 + 2*x*x)
        y_distorted = y * radial_distortion + p1*(r2 + 2*y*y) + 2*p2*x*y

        return np.column_stack([x_distorted, y_distorted])

    def _apply_intrinsic_matrix(self, points):
        """
        Apply camera intrinsic matrix to points
        """
        # Convert to homogeneous coordinates
        points_h = np.column_stack([points, np.ones(len(points))])

        # Apply camera matrix
        points_cam = points_h @ self.camera_matrix.T

        # Convert back to 2D
        points_2d = points_cam[:, :2] / points_cam[:, 2:3]

        return points_2d
```

## Exercises

### Exercise 1: Identify Sensor Components
Examine the following Gazebo sensor configuration and identify the key components:

```xml
<sensor name="multi_sensor" type="ray">
  <always_on>1</always_on>
  <update_rate>20</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="sensor_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
  </plugin>
</sensor>
```

What type of sensor is this? What are its key parameters? How would you modify it to increase resolution?

### Exercise 2: Sensor Fusion Implementation
Implement a simple sensor fusion algorithm that combines data from:
- An IMU providing orientation estimates
- A camera providing position measurements
- A LiDAR providing distance measurements

Create a class that maintains a state estimate and updates it based on sensor measurements.

## Summary

This chapter has covered the critical topic of sensor simulation in digital twin systems for humanoid robots. We explored various sensor types including LiDAR, depth cameras, IMUs, and force/torque sensors, examining how to configure them in simulation environments and how to add realistic noise and imperfections that characterize real hardware.

Sensor simulation is fundamental to creating effective digital twins because it provides the perceptual input that AI systems need to understand and interact with their environment. By accurately simulating sensor characteristics—including noise, bias, drift, and calibration parameters—we can train AI models that will perform effectively when deployed on real robots.

The sensor fusion techniques discussed demonstrate how multiple sensor modalities can be combined to create more robust and accurate perception systems. This is particularly important for humanoid robots, which require rich sensory input to navigate complex environments and perform dexterous manipulation tasks.

With the completion of this chapter, we have covered all three key components of digital twin systems: physics simulation, visual environments, and sensor simulation. These elements work together to create comprehensive virtual environments that can be used for AI development, testing, and validation before deployment on physical robots.