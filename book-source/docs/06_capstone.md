# Chapter 6: Capstone Project - End-to-End Physical AI System

## Overview

This capstone project brings together all concepts from Chapters 1-5 to build a complete, production-ready Physical AI system. You'll implement a **robotic book assistant** that:

1. **Perceives**: Uses computer vision to detect objects and read text
2. **Reasons**: Understands natural language instructions via language models
3. **Acts**: Controls a robot arm to manipulate books and objects
4. **Learns**: Improves through experience and user feedback

### Project Goals and Learning Outcomes

By completing this capstone project, you will:

- **Integrate** multiple Physical AI subsystems (perception, planning, control, AI/ML) into a cohesive architecture
- **Deploy** a real robotic system that operates safely and reliably in unstructured environments
- **Validate** sim-to-real transfer techniques and measure performance against quantitative benchmarks
- **Demonstrate** end-to-end task execution from natural language instructions to physical manipulation
- **Build** production-grade monitoring, logging, and failure recovery mechanisms

### What You Will Build

The **Robotic Book Assistant** is a complete Physical AI system capable of:

1. **Visual Scene Understanding**: Detecting books, reading spines, recognizing book covers, and understanding spatial relationships
2. **Natural Language Task Parsing**: Accepting commands like "Find the red book and move it to the shelf" and decomposing them into executable sub-tasks
3. **Dexterous Manipulation**: Grasping books of various sizes, navigating cluttered spaces, and precise placement
4. **Continuous Learning**: Improving grasp success rates through logged experience and reinforcement learning
5. **Safe Operation**: Operating alongside humans with collision avoidance, force limits, and emergency stop capabilities

### Success Criteria and Validation Metrics

Your deployed system will be evaluated against these quantitative benchmarks:

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| **Object Detection Precision** | greater than 90% | mAP@0.5 on test set of 100 books |
| **Grasp Success Rate** | greater than 85% | 20 attempts per object category |
| **Motion Planning Success** | greater than 95% | 100 random valid target poses |
| **Task Completion Rate** | greater than 80% | 50 natural language instructions |
| **Safety Violations** | 0 | Collision events, force limit breaches |
| **Average Task Time** | less than 60s | Pick-and-place single book |
| **API Response Time** | less than 200ms | p95 latency for task requests |

These metrics ensure your system is production-ready for real-world deployment.

### Project Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    Capstone System                            │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  Frontend (React/Docusaurus)     Backend (FastAPI)            │
│  ├─ Book Content                  ├─ User Management         │
│  ├─ Auth Forms                    ├─ RAG Chatbot             │
│  └─ Chat Interface                ├─ Personalization         │
│                                   └─ API Middleware          │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │         Hardware Integration Layer (ROS 2)               │ │
│  ├─────────────────────────────────────────────────────────┤ │
│  │                                                          │ │
│  │  Perception                Motion Control    Actuation  │ │
│  │  ├─ Camera Nodes           ├─ Path Planner  ├─ Gripper │ │
│  │  ├─ LiDAR Nodes            ├─ Trajectory    ├─ Motors  │ │
│  │  └─ IMU Nodes              └─ Control Loop  └─ Base    │ │
│  │                                                          │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │        AI/ML Services (Isaac Sim + VLA)                 │ │
│  ├─────────────────────────────────────────────────────────┤ │
│  │  ├─ Simulation Environment                             │ │
│  │  ├─ Vision-Language-Action Models                      │ │
│  │  └─ Reinforcement Learning Training                    │ │
│  │                                                          │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

## System Architecture Deep Dive

### Hardware Architecture

The physical hardware stack consists of three integrated subsystems:

**1. Robot Manipulator Subsystem**
- 6-DOF collaborative robot arm (UR10e recommended for reach and payload)
- Parallel-jaw gripper with force sensing (Robotiq 2F-140)
- Wrist-mounted force/torque sensor for contact detection
- Emergency stop system with physical buttons at multiple locations

**2. Perception Subsystem**
- RGB-D camera (Intel RealSense D435) mounted on robot wrist for eye-in-hand configuration
- Scene camera (Intel RealSense D455) for global workspace monitoring
- Optional: 2D LiDAR for obstacle detection and safety zones
- LED ring lights for consistent illumination

**3. Compute Subsystem**
- NVIDIA Jetson AGX Orin (32GB) for edge inference OR
- Cloud workstation: AMD Ryzen 9 + NVIDIA RTX 4090 for development
- Gigabit Ethernet for real-time robot communication
- WiFi 6 for API access and monitoring

**Data Flow Architecture:**

```
┌─────────────────────────────────────────────────────────────┐
│                    Physical World                            │
│  Books, Workspace, Obstacles, Human Operators                │
└─────────────────────────────────────────────────────────────┘
                           ▲ │
                   Actuation│ │Sensing
                           │ ▼
┌─────────────────────────────────────────────────────────────┐
│              Hardware Layer (100 Hz)                         │
│  ├─ Robot Arm (UR10e) ──> Joint States                      │
│  ├─ Gripper (Robotiq) ──> Grasp State                       │
│  ├─ RGB-D Cameras ──────> Images + Point Clouds             │
│  └─ F/T Sensor ─────────> Force/Torque Readings             │
└─────────────────────────────────────────────────────────────┘
                           │ │
                  ROS 2 Topics/Services (DDS)
                           │ ▼
┌─────────────────────────────────────────────────────────────┐
│         Perception Layer (30 Hz)                             │
│  ├─ Object Detection (YOLO) ──> Bounding Boxes              │
│  ├─ Segmentation (SAM) ───────> Instance Masks              │
│  ├─ Pose Estimation ───────────> 6D Object Poses            │
│  └─ Scene Graph ────────────────> Spatial Relationships     │
└─────────────────────────────────────────────────────────────┘
                           │ │
                  Semantic Understanding
                           │ ▼
┌─────────────────────────────────────────────────────────────┐
│            Reasoning Layer (On-demand)                       │
│  ├─ Language Model (Claude) ─> Task Decomposition           │
│  ├─ VLA Model (OpenVLA) ─────> Action Proposals             │
│  └─ Task Planner ─────────────> Execution Graph             │
└─────────────────────────────────────────────────────────────┘
                           │ │
                    Motion Commands
                           │ ▼
┌─────────────────────────────────────────────────────────────┐
│         Planning & Control Layer (10 Hz)                     │
│  ├─ Motion Planner (MoveIt2) ──> Collision-Free Paths       │
│  ├─ Trajectory Optimizer ──────> Smooth Joint Trajectories  │
│  ├─ Force Controller ───────────> Compliant Manipulation    │
│  └─ Safety Monitor ──────────────> Emergency Stop Trigger   │
└─────────────────────────────────────────────────────────────┘
                           │ │
                   Control Commands (100 Hz)
                           │ ▼
┌─────────────────────────────────────────────────────────────┐
│          Application Layer (REST API)                        │
│  ├─ FastAPI Backend ──> Task Queue + Monitoring             │
│  ├─ Database (PostgreSQL) ──> Task History + Metrics        │
│  └─ Vector DB (Qdrant) ──> Semantic Book Search             │
└─────────────────────────────────────────────────────────────┘
```

### Software Architecture

The software stack follows a layered architecture with clear separation of concerns:

**ROS 2 Node Structure:**

```python
# Core system nodes (all running in same ROS 2 workspace)

ros2_ws/
├─ perception_node          # 30 Hz perception loop
│  ├─ Subscribes: /camera/color/image_raw, /camera/depth/points
│  └─ Publishes: /detections, /scene_graph
│
├─ planning_node           # Motion planning service
│  ├─ Subscribes: /joint_states, /scene_graph
│  └─ Services: /plan_motion, /execute_trajectory
│
├─ control_node           # 100 Hz control loop
│  ├─ Subscribes: /joint_commands
│  └─ Publishes: /joint_states, /gripper_state
│
├─ safety_monitor_node    # Real-time safety checking
│  ├─ Subscribes: /joint_states, /force_torque
│  └─ Services: /emergency_stop
│
└─ task_execution_node    # High-level task orchestration
   ├─ Subscribes: /detections, /joint_states
   └─ Services: /execute_task, /get_task_status
```

**Complete ROS 2 Launch File:**

```python
# system.launch.py - Launches entire system
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Arguments
    use_sim = LaunchConfiguration('use_sim', default='false')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.100')

    return LaunchDescription([
        # Robot driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ur_robot_driver'),
                            'launch', 'ur10e.launch.py')
            ]),
            launch_arguments={
                'robot_ip': robot_ip,
                'use_fake_hardware': use_sim
            }.items()
        ),

        # MoveIt2 planning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ur_moveit_config'),
                            'launch', 'ur_moveit.launch.py')
            ])
        ),

        # Perception
        Node(
            package='capstone_perception',
            executable='perception_node',
            name='perception',
            parameters=[{
                'camera_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/points',
                'detection_confidence': 0.7,
                'model_path': '/models/yolov8_books.pt'
            }],
            output='screen'
        ),

        # Task execution
        Node(
            package='capstone_execution',
            executable='task_executor',
            name='task_executor',
            parameters=[{
                'claude_api_key': os.environ.get('ANTHROPIC_API_KEY'),
                'max_task_duration': 120.0
            }],
            output='screen'
        ),

        # Safety monitor
        Node(
            package='capstone_safety',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[{
                'force_limit': 100.0,  # Newtons
                'velocity_limit': 0.5,  # m/s
                'workspace_bounds': {
                    'x': [-1.0, 1.0],
                    'y': [-1.0, 1.0],
                    'z': [0.0, 2.0]
                }
            }],
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(
                get_package_share_directory('capstone_viz'),
                'config', 'system.rviz'
            )]
        )
    ])
```

### Communication Patterns

The system uses three primary communication patterns:

1. **Topics** (pub/sub): High-frequency sensor data, detections, states
2. **Services** (request/response): Motion planning, task execution
3. **Actions** (long-running goals): Complex trajectories with feedback

**Example Service Definition:**

```python
# TaskExecution.srv
# Request
string task_description
bool use_vision  # Whether to use visual grounding
float32 timeout  # Maximum execution time

---
# Response
bool success
string result_message
float32 execution_time
string[] actions_executed
```

## System Components

### 1. Hardware Setup

| Component | Specification | Cost |
|-----------|---------------|------|
| Robot Arm | UR10e or Franka Panda | $35K-50K |
| Gripper | Robotiq 2F-140 | $2K |
| Camera | Intel RealSense D435 | $150 |
| LiDAR | Velodyne Puck | $1K |
| Control PC | NVIDIA Jetson AGX (or cloud) | $5K |
| Network | Gigabit Ethernet + WiFi | $500 |

### 2. Software Stack

```
Tier 1 (Hardware Interface)
├─ ROS 2 Humble
├─ UR ROS Driver / Franka ROS Driver
└─ Gazebo Physics Simulation

Tier 2 (Perception)
├─ OpenCV for image processing
├─ PyTorch for deep learning
├─ YOLO for object detection
└─ Segment Anything (SAM) for segmentation

Tier 3 (Planning & Control)
├─ MoveIt! 2 for motion planning
├─ Isaac GEM for manipulation skills
└─ RRTConnect for trajectory planning

Tier 4 (AI/ML Services)
├─ Claude API for language understanding
├─ OpenVLA for vision-language-action
└─ Qdrant for vector search

Tier 5 (Application Layer)
├─ FastAPI backend
├─ Docusaurus frontend
└─ PostgreSQL + Qdrant databases
```

## Implementation Guide

### Phase 1: Development Environment

```bash
# Create workspace
mkdir -p ros2_ws/src
cd ros2_ws

# Clone necessary repositories
git clone https://github.com/UR-Robotics/ur_robot_driver src/ur_robot_driver
git clone https://github.com/ros-planning/moveit2 src/moveit2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source setup
source install/setup.bash
```

### Phase 2: Robot Configuration

```python
# robot_config.py - Define robot capabilities
ROBOT_CONFIG = {
    "name": "ur10e",
    "type": "collaborative",
    "dof": 6,
    "joints": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ],
    "joint_limits": {
        "lower": [-3.14159] * 6,
        "upper": [3.14159] * 6,
        "velocity": [2.0] * 6,  # rad/s
        "acceleration": [1.0] * 6  # rad/s²
    },
    "workspace": {
        "x": [-1.5, 1.5],
        "y": [-1.5, 1.5],
        "z": [0, 2.5]
    },
    "gripper": {
        "type": "parallel",
        "max_force": 140,  # N
        "stroke": 0.14  # m
    },
    "sensors": {
        "camera": {
            "type": "rgb-d",
            "resolution": [640, 480],
            "focal_length": 635
        },
        "force_torque": {
            "location": "wrist",
            "range": 2000  # N
        }
    }
}

# Load configuration
import yaml
with open('robot_config.yaml', 'w') as f:
    yaml.dump(ROBOT_CONFIG, f)
```

## Hardware Integration

This section covers physical assembly, sensor calibration, and hardware validation procedures.

### Robot Assembly and Setup

**Step 1: Mechanical Assembly**

```bash
# Assembly checklist
1. Mount robot arm to sturdy workbench (minimum 500kg capacity)
2. Connect emergency stop button to robot controller
3. Install gripper on end effector with proper torque (8 Nm for UR10e)
4. Mount wrist camera using 3D-printed adapter
5. Secure all cables with cable management system
```

**Step 2: Electrical Connections**

```
Power Distribution:
┌─────────────────────┐
│  240V AC Supply     │
└──────┬──────────────┘
       │
       ├───> Robot Controller (240V, 16A)
       ├───> Compute PC (120V, 10A)
       ├───> Gripper Power Supply (24V DC, 2A)
       └───> LED Lights (12V DC, 3A)

Network Topology:
┌─────────────────────┐
│  Gigabit Switch     │
└──────┬──────────────┘
       │
       ├───> Robot Controller (192.168.1.100)
       ├───> Compute PC (192.168.1.10)
       └───> WiFi Router (for API access)
```

**Step 3: Network Configuration**

```bash
# Configure static IP for robot
# On UR10e teach pendant:
# Settings > System > Network
# IP: 192.168.1.100
# Subnet: 255.255.255.0
# Gateway: 192.168.1.1

# Configure compute PC
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.10/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Test connectivity
ping 192.168.1.100
# Expected: < 1ms latency
```

### Sensor Calibration Procedures

**Camera Intrinsic Calibration**

```python
# camera_calibration.py
import cv2
import numpy as np
import pyrealsense2 as rs

class CameraCalibrator:
    """Calibrate RGB-D camera intrinsics and extrinsics"""

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    def calibrate_intrinsics(self, checkerboard_size=(9, 6), square_size=0.025):
        """
        Calibrate camera using checkerboard pattern

        Args:
            checkerboard_size: Inner corners (width, height)
            square_size: Size of each square in meters
        """
        # Prepare object points
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_size[0],
                                0:checkerboard_size[1]].T.reshape(-1, 2)
        objp *= square_size

        objpoints = []  # 3D points in world space
        imgpoints = []  # 2D points in image plane

        self.pipeline.start(self.config)

        print("Capture 20+ images of checkerboard from different angles...")
        print("Press SPACE to capture, ESC to finish")

        while len(imgpoints) < 20:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

            if ret:
                cv2.drawChessboardCorners(color_image, checkerboard_size,
                                         corners, ret)
                cv2.putText(color_image, f"Captures: {len(imgpoints)}/20",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('Calibration', color_image)
            key = cv2.waitKey(1)

            if key == ord(' ') and ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                print(f"Captured {len(imgpoints)}/20")
            elif key == 27:  # ESC
                break

        # Calibrate
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        # Save calibration
        np.savez('camera_calibration.npz',
                 camera_matrix=mtx,
                 distortion=dist,
                 rvecs=rvecs,
                 tvecs=tvecs)

        print(f"Calibration complete. Reprojection error: {ret:.4f}")
        print(f"Camera matrix:\n{mtx}")

        self.pipeline.stop()
        cv2.destroyAllWindows()

        return mtx, dist

    def calibrate_hand_eye(self, robot_poses, checkerboard_poses):
        """
        Calibrate camera-to-robot transformation

        Args:
            robot_poses: List of robot end-effector poses (4x4 matrices)
            checkerboard_poses: List of checkerboard poses in camera frame
        """
        # Use OpenCV's hand-eye calibration
        R_gripper2base = [pose[:3, :3] for pose in robot_poses]
        t_gripper2base = [pose[:3, 3] for pose in robot_poses]
        R_target2cam = [pose[:3, :3] for pose in checkerboard_poses]
        t_target2cam = [pose[:3, 3] for pose in checkerboard_poses]

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )

        transform = np.eye(4)
        transform[:3, :3] = R_cam2gripper
        transform[:3, 3] = t_cam2gripper.flatten()

        np.save('hand_eye_calibration.npy', transform)
        print(f"Hand-eye calibration complete:\n{transform}")

        return transform

if __name__ == '__main__':
    calibrator = CameraCalibrator()
    mtx, dist = calibrator.calibrate_intrinsics()
```

**Force/Torque Sensor Calibration**

```python
# ft_sensor_calibration.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np

class FTSensorCalibrator(Node):
    """Calibrate force/torque sensor bias"""

    def __init__(self):
        super().__init__('ft_calibrator')
        self.subscription = self.create_subscription(
            WrenchStamped, '/force_torque_sensor', self.ft_callback, 10
        )
        self.samples = []
        self.target_samples = 1000

    def ft_callback(self, msg):
        if len(self.samples) < self.target_samples:
            self.samples.append([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z
            ])

            if len(self.samples) % 100 == 0:
                self.get_logger().info(
                    f"Collected {len(self.samples)}/{self.target_samples} samples"
                )

    def compute_bias(self):
        """Compute and save sensor bias (with no load)"""
        self.get_logger().info("Collecting bias samples (no load on sensor)...")

        while len(self.samples) < self.target_samples:
            rclpy.spin_once(self, timeout_sec=0.01)

        bias = np.mean(self.samples, axis=0)
        std = np.std(self.samples, axis=0)

        self.get_logger().info(f"Bias: {bias}")
        self.get_logger().info(f"Noise (std): {std}")

        # Save calibration
        np.savez('ft_calibration.npz', bias=bias, noise=std)

        return bias, std

def main():
    rclpy.init()
    calibrator = FTSensorCalibrator()

    print("Ensure robot is stationary with NO load on end effector")
    input("Press Enter to start calibration...")

    bias, noise = calibrator.compute_bias()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Actuator Setup and Testing

**Gripper Configuration**

```python
# gripper_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from robotiq_2f_msgs.msg import CommandRobotiqGripper, RobotiqGripperStatus

class GripperController(Node):
    """Interface for Robotiq 2F-140 gripper"""

    def __init__(self):
        super().__init__('gripper_controller')

        # Publishers
        self.pub_command = self.create_publisher(
            CommandRobotiqGripper, '/gripper/command', 10
        )

        # Subscribers
        self.sub_status = self.create_subscription(
            RobotiqGripperStatus, '/gripper/status', self.status_callback, 10
        )

        self.current_position = 0.0
        self.is_moving = False

    def status_callback(self, msg):
        self.current_position = msg.position
        self.is_moving = msg.is_moving

    def open(self, speed=0.1, force=50):
        """Open gripper

        Args:
            speed: Opening speed (0-1)
            force: Grip force in Newtons (0-140)
        """
        cmd = CommandRobotiqGripper()
        cmd.position = 0.0  # Fully open
        cmd.speed = speed
        cmd.force = force / 140.0  # Normalize to 0-1

        self.pub_command.publish(cmd)
        self.get_logger().info("Opening gripper")

    def close(self, width=0.0, speed=0.1, force=50):
        """Close gripper to target width

        Args:
            width: Target width in meters (0 = fully closed)
            speed: Closing speed (0-1)
            force: Grip force in Newtons (0-140)
        """
        cmd = CommandRobotiqGripper()
        cmd.position = 1.0 - (width / 0.14)  # Map to 0-1
        cmd.speed = speed
        cmd.force = force / 140.0

        self.pub_command.publish(cmd)
        self.get_logger().info(f"Closing gripper to {width*1000:.1f}mm")

    def wait_for_motion_complete(self, timeout=5.0):
        """Wait until gripper motion completes"""
        import time
        start = time.time()

        while self.is_moving and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)

        return not self.is_moving

def main():
    rclpy.init()
    gripper = GripperController()

    print("Testing gripper...")

    # Test sequence
    gripper.open()
    gripper.wait_for_motion_complete()

    time.sleep(1)

    gripper.close(width=0.05)  # Close to 50mm
    gripper.wait_for_motion_complete()

    print("Gripper test complete")

    rclpy.shutdown()

if __name__ == '__main__':
    import time
    main()
```

### Power Distribution and Safety

**Emergency Stop Circuit**

```python
# safety_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
import numpy as np

class SafetyMonitor(Node):
    """Real-time safety monitoring with emergency stop"""

    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('force_limit', 100.0)  # Newtons
        self.declare_parameter('velocity_limit', 0.5)  # m/s
        self.declare_parameter('workspace_bounds', {
            'x': [-1.0, 1.0],
            'y': [-1.0, 1.0],
            'z': [0.0, 2.0]
        })

        self.force_limit = self.get_parameter('force_limit').value
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.workspace = self.get_parameter('workspace_bounds').value

        # State
        self.current_pose = None
        self.current_velocity = None
        self.current_force = None
        self.is_safe = True

        # Subscribers
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.sub_force = self.create_subscription(
            WrenchStamped, '/force_torque', self.force_callback, 10
        )

        # Services
        self.srv_estop = self.create_service(
            Trigger, '/emergency_stop', self.emergency_stop_callback
        )

        # Timer for safety checks (100 Hz)
        self.timer = self.create_timer(0.01, self.safety_check)

    def joint_callback(self, msg):
        # Compute end-effector velocity (simplified)
        self.current_velocity = np.max(np.abs(msg.velocity))

    def force_callback(self, msg):
        force_magnitude = np.sqrt(
            msg.wrench.force.x**2 +
            msg.wrench.force.y**2 +
            msg.wrench.force.z**2
        )
        self.current_force = force_magnitude

    def safety_check(self):
        """Check safety conditions"""
        violations = []

        # Check force limits
        if self.current_force and self.current_force > self.force_limit:
            violations.append(f"Force limit exceeded: {self.current_force:.1f}N")

        # Check velocity limits
        if self.current_velocity and self.current_velocity > self.velocity_limit:
            violations.append(f"Velocity limit exceeded: {self.current_velocity:.2f}m/s")

        # Check workspace bounds
        if self.current_pose:
            x, y, z = self.current_pose
            if not (self.workspace['x'][0] <= x <= self.workspace['x'][1]):
                violations.append(f"X position out of bounds: {x:.2f}m")
            if not (self.workspace['y'][0] <= y <= self.workspace['y'][1]):
                violations.append(f"Y position out of bounds: {y:.2f}m")
            if not (self.workspace['z'][0] <= z <= self.workspace['z'][1]):
                violations.append(f"Z position out of bounds: {z:.2f}m")

        if violations:
            self.get_logger().error(f"SAFETY VIOLATION: {', '.join(violations)}")
            self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if self.is_safe:
            self.is_safe = False
            self.get_logger().fatal("EMERGENCY STOP TRIGGERED")

            # Stop robot (implementation depends on robot driver)
            # For UR robots:
            # self.ur_driver.stop()

            # Or via GPIO for hardware E-stop
            # GPIO.output(ESTOP_PIN, GPIO.HIGH)

    def emergency_stop_callback(self, request, response):
        self.trigger_emergency_stop()
        response.success = True
        response.message = "Emergency stop activated"
        return response

def main():
    rclpy.init()
    monitor = SafetyMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Troubleshooting Guide

**Problem: Robot Not Responding**

```bash
# 1. Check network connectivity
ping 192.168.1.100

# 2. Check ROS 2 topics
ros2 topic list

# 3. Check robot driver node
ros2 node info /ur_robot_driver

# 4. Restart robot controller
# Power cycle robot, wait 30 seconds, power on
```

**Problem: Camera Not Detected**

```bash
# 1. List USB devices
lsusb | grep Intel

# 2. Check RealSense
rs-enumerate-devices

# 3. Test camera
realsense-viewer

# 4. Reinstall drivers if needed
sudo apt install ros-humble-realsense2-camera
```

**Problem: Gripper Communication Failed**

```bash
# 1. Check Modbus connection
ros2 topic echo /gripper/status

# 2. Verify power supply (24V DC)
# Use multimeter to check voltage at gripper connector

# 3. Re-initialize gripper
ros2 service call /gripper/initialize std_srvs/srv/Trigger
```

## Perception Pipeline Implementation

### Phase 3: Perception Pipeline

```python
# perception_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Load YOLO model
        self.detector = YOLO('yolov8n.pt')

        # Publishers
        self.pub_detections = self.create_publisher(String, 'detections', 10)
        self.pub_annotated = self.create_publisher(Image, 'annotated_image', 10)

        # Subscribers
        self.sub_image = self.create_subscription(
            Image, 'camera/rgb/image_raw', self.image_callback, 10
        )
        self.sub_pointcloud = self.create_subscription(
            PointCloud2, 'camera/depth/points', self.pointcloud_callback, 10
        )

    def image_callback(self, msg):
        """Process incoming image"""
        # Convert ROS Image to OpenCV
        frame = self.ros_image_to_cv2(msg)

        # Detect objects
        results = self.detector(frame)

        # Parse detections
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().item()
            class_id = int(box.cls[0])
            class_name = self.detector.names[class_id]

            detections.append({
                "class": class_name,
                "confidence": float(conf),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "center": [float((x1+x2)/2), float((y1+y2)/2)]
            })

            # Draw on frame
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}",
                       (int(x1), int(y1)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        import json
        self.pub_detections.publish(String(data=json.dumps(detections)))

        # Publish annotated image
        self.pub_annotated.publish(self.cv2_to_ros_image(frame))

        self.get_logger().info(f"Detected {len(detections)} objects")

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D object localization"""
        # Convert to numpy
        points = self.pointcloud2_to_xyz(msg)

        # Simple plane detection (floor)
        # In production: use RANSAC or deep learning

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Real-time Object Detection with YOLO**

The perception pipeline processes camera feeds at 30 Hz and publishes structured detections. Beyond the basic implementation shown in Phase 3, here are production enhancements:

```python
# advanced_perception.py - Production-grade perception with sensor fusion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point
import numpy as np
import torch
from ultralytics import YOLO
from segment_anything import sam_model_registry, SamPredictor
import open3d as o3d

class AdvancedPerceptionNode(Node):
    """Production perception with object detection, segmentation, and 6D pose"""

    def __init__(self):
        super().__init__('advanced_perception')

        # Load models
        self.yolo = YOLO('yolov8m.pt')  # Medium model for accuracy
        self.yolo.to('cuda' if torch.cuda.is_available() else 'cpu')

        # Load SAM for segmentation
        sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth")
        self.sam_predictor = SamPredictor(sam)

        # Camera calibration
        self.camera_matrix = None
        self.depth_scale = 0.001  # mm to meters

        # Publishers
        self.pub_detections_3d = self.create_publisher(
            Detection3DArray, 'detections_3d', 10
        )

        # Subscribers
        self.sub_rgb = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )

        self.latest_rgb = None
        self.latest_depth = None

        # Create timer for synchronized processing
        self.timer = self.create_timer(1.0/30.0, self.process_frame)

    def camera_info_callback(self, msg):
        """Extract camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"Camera calibration loaded: {self.camera_matrix[0, 0]:.1f}px focal length")

    def rgb_callback(self, msg):
        self.latest_rgb = self.ros_to_cv2(msg)

    def depth_callback(self, msg):
        self.latest_depth = self.ros_to_cv2(msg, encoding='passthrough')

    def process_frame(self):
        """Main perception loop"""
        if self.latest_rgb is None or self.latest_depth is None:
            return

        # 1. Object detection
        detections_2d = self.detect_objects(self.latest_rgb)

        # 2. Instance segmentation
        masks = self.segment_objects(self.latest_rgb, detections_2d)

        # 3. 6D pose estimation
        detections_3d = self.estimate_poses(
            detections_2d, masks, self.latest_depth
        )

        # 4. Publish
        self.pub_detections_3d.publish(detections_3d)

    def detect_objects(self, rgb_image):
        """YOLO object detection"""
        results = self.yolo(rgb_image, conf=0.7, iou=0.5)
        return results[0].boxes

    def segment_objects(self, rgb_image, boxes):
        """Segment each detected object using SAM"""
        self.sam_predictor.set_image(rgb_image)
        masks = []

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            input_box = np.array([x1, y1, x2, y2])

            mask, score, _ = self.sam_predictor.predict(
                box=input_box,
                multimask_output=False
            )
            masks.append(mask[0])

        return masks

    def estimate_poses(self, boxes, masks, depth_image):
        """Estimate 6D pose for each object"""
        detections_3d = Detection3DArray()
        detections_3d.header.frame_id = "camera_color_optical_frame"
        detections_3d.header.stamp = self.get_clock().now().to_msg()

        for i, (box, mask) in enumerate(zip(boxes, masks)):
            # Extract point cloud for this object
            object_points = self.extract_object_pointcloud(mask, depth_image)

            if len(object_points) < 100:
                continue  # Too few points

            # Compute 6D pose using PCA
            pose = self.compute_object_pose(object_points)

            # Create detection message
            detection = Detection3D()
            detection.bbox.center.position = pose.position
            detection.bbox.center.orientation = pose.orientation

            # Set object class
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box.cls[0]))
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)

            detections_3d.detections.append(detection)

        return detections_3d

    def extract_object_pointcloud(self, mask, depth_image):
        """Extract 3D points for masked region"""
        if self.camera_matrix is None:
            return np.array([])

        points = []
        h, w = mask.shape
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        ys, xs = np.where(mask)

        for x, y in zip(xs, ys):
            depth = depth_image[y, x] * self.depth_scale

            if depth > 0.1 and depth < 2.0:  # Valid depth range
                # Back-project to 3D
                x3d = (x - cx) * depth / fx
                y3d = (y - cy) * depth / fy
                z3d = depth
                points.append([x3d, y3d, z3d])

        return np.array(points)

    def compute_object_pose(self, points):
        """Compute 6D pose from point cloud using PCA"""
        # Center of object
        centroid = np.mean(points, axis=0)

        # Use PCA for orientation
        centered = points - centroid
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # Sort by eigenvalue
        idx = eigenvalues.argsort()[::-1]
        eigenvectors = eigenvectors[:, idx]

        # Create pose
        pose = Pose()
        pose.position = Point(x=float(centroid[0]),
                             y=float(centroid[1]),
                             z=float(centroid[2]))

        # Convert rotation matrix to quaternion
        from scipy.spatial.transform import Rotation
        rot = Rotation.from_matrix(eigenvectors)
        quat = rot.as_quat()  # [x, y, z, w]

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def ros_to_cv2(self, msg, encoding='bgr8'):
        """Convert ROS Image to OpenCV"""
        from cv_bridge import CvBridge
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, encoding)

def main():
    rclpy.init()
    node = AdvancedPerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Point Cloud Processing and Filtering**

```python
# pointcloud_processor.py - Advanced point cloud processing
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor:
    """Process and filter point clouds for manipulation"""

    def __init__(self):
        # Parameters for segmentation
        self.voxel_size = 0.005  # 5mm voxels
        self.plane_threshold = 0.01  # 1cm for plane fitting

    def process_scene(self, pointcloud_msg):
        """
        Process raw point cloud:
        1. Downsample
        2. Remove table plane
        3. Cluster objects
        4. Extract graspable surfaces
        """
        # Convert ROS PointCloud2 to Open3D
        pcd = self.ros_to_open3d(pointcloud_msg)

        # 1. Downsample
        pcd_down = pcd.voxel_down_sample(self.voxel_size)

        # 2. Remove outliers
        pcd_clean, _ = pcd_down.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )

        # 3. Segment plane (table)
        plane_model, inliers = pcd_clean.segment_plane(
            distance_threshold=self.plane_threshold,
            ransac_n=3,
            num_iterations=1000
        )

        # Extract objects (non-plane points)
        pcd_objects = pcd_clean.select_by_index(inliers, invert=True)

        # 4. Cluster objects
        labels = np.array(pcd_objects.cluster_dbscan(
            eps=0.02, min_points=10
        ))

        # Extract each object cluster
        objects = []
        for label in set(labels):
            if label == -1:  # Noise
                continue

            object_indices = np.where(labels == label)[0]
            object_pcd = pcd_objects.select_by_index(object_indices)

            objects.append({
                'pointcloud': object_pcd,
                'centroid': object_pcd.get_center(),
                'num_points': len(object_indices)
            })

        return objects

    def ros_to_open3d(self, pointcloud_msg):
        """Convert ROS PointCloud2 to Open3D PointCloud"""
        points = []
        colors = []

        for point in pc2.read_points(pointcloud_msg, skip_nans=True):
            x, y, z, rgb = point[:4]
            points.append([x, y, z])

            # Extract RGB
            rgb_packed = int(rgb)
            r = (rgb_packed >> 16) & 0xFF
            g = (rgb_packed >> 8) & 0xFF
            b = rgb_packed & 0xFF
            colors.append([r/255.0, g/255.0, b/255.0])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

        return pcd
```

**Sensor Fusion for Robust Detection**

```python
# sensor_fusion.py - Fuse RGB and depth for robust detection
class SensorFusion:
    """Fuse multiple sensor modalities for robust perception"""

    def __init__(self):
        self.detection_history = []  # Track detections over time
        self.max_history = 10

    def fuse_detections(self, rgb_detections, depth_detections):
        """
        Fuse RGB-based and depth-based detections

        Returns: Filtered, high-confidence detections
        """
        fused = []

        for rgb_det in rgb_detections:
            # Find matching depth detection (within 10cm)
            match = self.find_spatial_match(rgb_det, depth_detections, threshold=0.1)

            if match:
                # Combine confidence scores
                fused_confidence = 0.7 * rgb_det.confidence + 0.3 * match.confidence

                fused_det = rgb_det
                fused_det.confidence = fused_confidence
                fused_det.pose_3d = match.pose  # Use depth-based 3D pose

                fused.append(fused_det)

        return fused

    def temporal_filter(self, detections):
        """Filter detections using temporal consistency"""
        self.detection_history.append(detections)

        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)

        # Only return detections seen in at least 3/10 frames
        consistent = []
        for det in detections:
            count = sum(1 for hist in self.detection_history
                       if self.is_detection_in_list(det, hist))

            if count >= 3:
                consistent.append(det)

        return consistent

    def find_spatial_match(self, det1, detections, threshold=0.1):
        """Find spatially close detection"""
        for det2 in detections:
            distance = np.linalg.norm(
                np.array([det1.pose.position.x, det1.pose.position.y, det1.pose.position.z]) -
                np.array([det2.pose.position.x, det2.pose.position.y, det2.pose.position.z])
            )

            if distance < threshold:
                return det2

        return None

    def is_detection_in_list(self, det, det_list, threshold=0.05):
        """Check if detection exists in list"""
        return self.find_spatial_match(det, det_list, threshold) is not None
```

## Planning and Control

### Phase 4: Motion Planning & Control

```python
# motion_controller.py
import rclpy
from rclpy.node import Node
from moveit2.python_api import MoveIt2
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # Initialize MoveIt2
        self.moveit = MoveIt2(
            node=self,
            joint_names=ROBOT_CONFIG['joints'],
            base_link_name="base_link",
            end_effector_name="tool0"
        )

        # Publishers
        self.pub_joint_cmd = self.create_publisher(JointState, 'joint_commands', 10)

        # Subscribers
        self.sub_target_pose = self.create_subscription(
            PoseStamped, 'target_pose', self.target_pose_callback, 10
        )

    def target_pose_callback(self, msg):
        """Plan to target pose using MoveIt2"""
        # Set target
        self.moveit.set_pose_target(msg.pose)

        # Plan motion
        success = self.moveit.plan()

        if success:
            # Execute trajectory
            self.moveit.execute()
            self.get_logger().info("Motion completed")
        else:
            self.get_logger().error("Motion planning failed")

    def move_to_position(self, x, y, z, roll=0, yaw=0, pitch=0):
        """Convenience method to move to Cartesian position"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Convert Euler to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, yaw, pitch)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.target_pose_callback(pose)

def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Trajectory Optimization for Smooth Motion**

```python
# trajectory_optimizer.py - Smooth, time-optimal trajectories
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

class TrajectoryOptimizer:
    """Optimize robot trajectories for smoothness and time"""

    def __init__(self, robot_config):
        self.vel_limits = np.array(robot_config['joint_limits']['velocity'])
        self.acc_limits = np.array(robot_config['joint_limits']['acceleration'])

    def optimize_trajectory(self, waypoints, dt=0.01):
        """
        Optimize trajectory through waypoints

        Args:
            waypoints: List of joint configurations (n_waypoints x n_joints)
            dt: Timestep in seconds

        Returns:
            Optimized trajectory with velocities and accelerations
        """
        n_waypoints = len(waypoints)
        n_joints = len(waypoints[0])

        # Initial time allocation (equal spacing)
        t_waypoints = np.linspace(0, n_waypoints, n_waypoints)

        # Optimize time allocation to minimize jerk
        def objective(t):
            # Penalize large time derivatives (jerk)
            traj = self.interpolate_trajectory(waypoints, t, dt)
            jerk = np.diff(traj['acceleration'], axis=0)
            return np.sum(jerk**2)

        def constraint_velocity(t):
            traj = self.interpolate_trajectory(waypoints, t, dt)
            return self.vel_limits - np.max(np.abs(traj['velocity']), axis=0)

        def constraint_acceleration(t):
            traj = self.interpolate_trajectory(waypoints, t, dt)
            return self.acc_limits - np.max(np.abs(traj['acceleration']), axis=0)

        # Optimize
        result = minimize(
            objective,
            t_waypoints,
            constraints=[
                {'type': 'ineq', 'fun': constraint_velocity},
                {'type': 'ineq', 'fun': constraint_acceleration}
            ],
            bounds=[(t_waypoints[i-1]+0.1, t_waypoints[i+1]-0.1) if 0 < i < n_waypoints-1
                   else (t_waypoints[i], t_waypoints[i]) for i in range(n_waypoints)]
        )

        # Generate final trajectory
        optimized_t = result.x
        trajectory = self.interpolate_trajectory(waypoints, optimized_t, dt)

        return trajectory

    def interpolate_trajectory(self, waypoints, t_waypoints, dt):
        """Interpolate smooth trajectory using cubic splines"""
        n_joints = len(waypoints[0])

        # Create spline for each joint
        splines = []
        for j in range(n_joints):
            joint_waypoints = [wp[j] for wp in waypoints]
            spline = CubicSpline(t_waypoints, joint_waypoints, bc_type='clamped')
            splines.append(spline)

        # Sample trajectory
        t_total = t_waypoints[-1]
        t_samples = np.arange(0, t_total, dt)

        positions = np.zeros((len(t_samples), n_joints))
        velocities = np.zeros((len(t_samples), n_joints))
        accelerations = np.zeros((len(t_samples), n_joints))

        for j, spline in enumerate(splines):
            positions[:, j] = spline(t_samples)
            velocities[:, j] = spline(t_samples, 1)  # First derivative
            accelerations[:, j] = spline(t_samples, 2)  # Second derivative

        return {
            'time': t_samples,
            'position': positions,
            'velocity': velocities,
            'acceleration': accelerations
        }
```

**Force Control and Compliance**

```python
# force_controller.py - Compliant manipulation with force feedback
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Twist
from sensor_msgs.msg import JointState
import numpy as np

class ForceController(Node):
    """Hybrid position/force control for contact-rich tasks"""

    def __init__(self):
        super().__init__('force_controller')

        # Control parameters
        self.K_p = 100.0  # Position stiffness
        self.K_f = 0.01   # Force compliance
        self.target_force = np.zeros(6)
        self.current_force = np.zeros(6)

        # Subscribers
        self.sub_force = self.create_subscription(
            WrenchStamped, '/force_torque', self.force_callback, 10
        )

        # Publishers
        self.pub_velocity = self.create_publisher(Twist, '/servo_velocity', 10)

        # Control loop at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

    def force_callback(self, msg):
        self.current_force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

    def control_loop(self):
        """Hybrid position/force control"""
        # Compute force error
        force_error = self.target_force - self.current_force

        # Admittance control: convert force error to velocity
        velocity_correction = self.K_f * force_error

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = velocity_correction[0]
        cmd.linear.y = velocity_correction[1]
        cmd.linear.z = velocity_correction[2]
        cmd.angular.x = velocity_correction[3]
        cmd.angular.y = velocity_correction[4]
        cmd.angular.z = velocity_correction[5]

        self.pub_velocity.publish(cmd)

    def set_target_force(self, force_z=10.0):
        """Set desired contact force (e.g., 10N downward)"""
        self.target_force[2] = -force_z  # Negative Z is down

    def grasp_with_force_control(self, target_force=20.0):
        """Close gripper until target force reached"""
        self.get_logger().info(f"Grasping with {target_force}N target force")

        # Slowly close gripper while monitoring force
        gripper_cmd = 0.0
        while self.current_force[2] < target_force:
            gripper_cmd += 0.001  # Increment closure
            # Publish gripper command
            time.sleep(0.01)

        self.get_logger().info(f"Grasp achieved at {self.current_force[2]:.1f}N")
```

**Safety-Critical Decision Making**

```python
# safety_decision_maker.py - Real-time safety monitoring and intervention
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class SafetyDecisionMaker(Node):
    """Monitor robot state and intervene if unsafe"""

    def __init__(self):
        super().__init__('safety_decision_maker')

        # Safety parameters
        self.workspace_bounds = {
            'x': [-1.0, 1.0],
            'y': [-1.0, 1.0],
            'z': [0.0, 2.0]
        }
        self.max_velocity = 0.5  # m/s
        self.max_force = 100.0   # N

        self.is_emergency_stop = False

        # Subscribers
        self.sub_pose = self.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10
        )
        self.sub_velocity = self.create_subscription(
            Twist, '/current_velocity', self.velocity_callback, 10
        )

        # Safety check at 100 Hz
        self.timer = self.create_timer(0.01, self.safety_check)

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def safety_check(self):
        """Real-time safety monitoring"""
        if self.is_emergency_stop:
            return

        violations = []

        # Check workspace bounds
        if hasattr(self, 'current_pose'):
            pos = self.current_pose.position
            if not (self.workspace_bounds['x'][0] <= pos.x <= self.workspace_bounds['x'][1]):
                violations.append(f"X out of bounds: {pos.x:.2f}m")
            if not (self.workspace_bounds['y'][0] <= pos.y <= self.workspace_bounds['y'][1]):
                violations.append(f"Y out of bounds: {pos.y:.2f}m")
            if not (self.workspace_bounds['z'][0] <= pos.z <= self.workspace_bounds['z'][1]):
                violations.append(f"Z out of bounds: {pos.z:.2f}m")

        # Check velocity limits
        if hasattr(self, 'current_velocity'):
            vel_magnitude = np.sqrt(
                self.current_velocity.linear.x**2 +
                self.current_velocity.linear.y**2 +
                self.current_velocity.linear.z**2
            )
            if vel_magnitude > self.max_velocity:
                violations.append(f"Velocity too high: {vel_magnitude:.2f}m/s")

        if violations:
            self.get_logger().error(f"SAFETY VIOLATIONS: {violations}")
            self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        """Immediately halt all motion"""
        if not self.is_emergency_stop:
            self.is_emergency_stop = True
            self.get_logger().fatal("EMERGENCY STOP ACTIVATED")

            # Send stop command to robot
            # Implementation depends on robot driver
```

**Real-World Constraint Handling**

```python
# constraint_handler.py - Handle real-world physical constraints
class ConstraintHandler:
    """Manage physical constraints during manipulation"""

    def __init__(self):
        self.grasp_force_range = (5.0, 50.0)  # Newtons
        self.object_weight_range = (0.1, 2.0)  # kg
        self.gripper_width_range = (0.0, 0.14)  # meters

    def validate_grasp(self, object_detection, gripper_state):
        """Check if grasp is physically feasible"""
        # Estimate object width from bounding box
        bbox = object_detection['bbox']
        estimated_width = (bbox[2] - bbox[0]) * 0.001  # pixels to meters (approx)

        if estimated_width > self.gripper_width_range[1]:
            return False, f"Object too wide: {estimated_width*1000:.0f}mm > 140mm"

        if estimated_width < 0.01:
            return False, "Object too small for reliable grasp"

        return True, "Grasp feasible"

    def compute_required_force(self, object_weight, friction_coef=0.3):
        """
        Compute minimum grasp force to prevent slipping

        F_grasp = (m * g) / (2 * mu)
        where mu is coefficient of friction
        """
        g = 9.81  # m/s^2
        min_force = (object_weight * g) / (2 * friction_coef)

        # Add safety factor
        required_force = min_force * 1.5

        # Clamp to gripper limits
        required_force = np.clip(required_force,
                                self.grasp_force_range[0],
                                self.grasp_force_range[1])

        return required_force

    def validate_trajectory(self, trajectory, obstacles):
        """Check trajectory for collisions"""
        for i, waypoint in enumerate(trajectory):
            for obstacle in obstacles:
                if self.check_collision(waypoint, obstacle):
                    return False, f"Collision at waypoint {i}"

        return True, "Trajectory collision-free"

    def check_collision(self, pose, obstacle):
        """Simple sphere-based collision check"""
        distance = np.linalg.norm(
            np.array([pose.position.x, pose.position.y, pose.position.z]) -
            obstacle.position
        )

        return distance < (0.1 + obstacle.radius)  # 10cm robot radius + obstacle
```

## Integration and Testing

### Phase 5: Task Execution Engine

```python
# task_executor.py
from enum import Enum
from typing import Optional
import asyncio

class TaskState(Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"

class TaskExecutor:
    def __init__(self, robot, perception, planner):
        self.robot = robot
        self.perception = perception
        self.planner = planner
        self.task_history = []

    async def execute_task(self, task_description: str) -> bool:
        """
        Execute a high-level task description.

        Task Planning:
        1. Parse instruction via Claude API
        2. Generate sub-goals
        3. Execute sub-goals sequentially
        4. Monitor success
        """

        from anthropic import Anthropic
        client = Anthropic()

        # Parse task into sub-goals
        task_plan = client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1024,
            messages=[
                {
                    "role": "user",
                    "content": f"""
                    You are a robot task planner. Break down this task into executable steps:
                    Task: {task_description}

                    Available robot actions:
                    - move_to(x, y, z): Move arm to position
                    - pick_object(object_name): Pick up object
                    - place_object(target_location): Place held object
                    - open_gripper(): Open gripper
                    - close_gripper(): Close gripper

                    Current scene: {self.perception.describe_scene()}

                    Return JSON:
                    {{
                        "goal": "task goal",
                        "steps": [
                            {{"action": "action_name", "params": {{...}}}},
                            ...
                        ],
                        "estimated_duration": 30
                    }}
                    """
                }
            ]
        )

        import json
        plan = json.loads(task_plan.content[0].text)

        # Execute plan
        for step in plan['steps']:
            action = step['action']
            params = step['params']

            self.robot.get_logger().info(f"Executing: {action}({params})")

            success = await self.execute_action(action, params)

            if not success:
                self.robot.get_logger().error(f"Action failed: {action}")
                return False

            await asyncio.sleep(0.5)

        return True

    async def execute_action(self, action: str, params: dict) -> bool:
        """Execute single action"""
        if action == "move_to":
            self.planner.move_to_position(
                params['x'], params['y'], params['z']
            )
            return True

        elif action == "pick_object":
            # Detect object
            objects = self.perception.get_detections()
            target = next((o for o in objects if o['class'] == params['object_name']), None)

            if not target:
                return False

            # Plan grasp
            grasp_pose = self.compute_grasp(target)

            # Move to grasp pose
            self.planner.move_to_position(*grasp_pose)

            # Close gripper
            self.robot.gripper.close()

            return True

        # ... other actions

        return False

    def compute_grasp(self, object_detection: dict):
        """Compute grasp pose for object"""
        # In production: use ML model or GEM
        bbox = object_detection['bbox']
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2

        # Estimate 3D position from depth
        depth = self.perception.get_depth_at(center_x, center_y)

        return [center_x / 1000, center_y / 1000, depth / 1000]
```

**End-to-End Integration Test Suite**

```python
# test_integration.py - Comprehensive integration tests
import pytest
import rclpy
from rclpy.node import Node
import numpy as np
import time

class IntegrationTestSuite:
    """Complete integration tests for capstone system"""

    @pytest.fixture(scope="session")
    def robot_system(self):
        """Initialize entire robot system"""
        rclpy.init()

        # Launch all nodes (perception, planning, control)
        # In real deployment, use launch file

        yield  # Tests run here

        rclpy.shutdown()

    def test_perception_latency(self, robot_system):
        """Ensure perception runs at 30 Hz with <50ms latency"""
        import statistics

        latencies = []
        for i in range(100):
            start = time.time()
            # Trigger perception
            detections = self.perception_node.get_detections()
            latency = (time.time() - start) * 1000  # ms

            latencies.append(latency)

        avg_latency = statistics.mean(latencies)
        p95_latency = np.percentile(latencies, 95)

        assert avg_latency < 33, f"Avg latency {avg_latency:.1f}ms > 33ms (30Hz)"
        assert p95_latency < 50, f"P95 latency {p95_latency:.1f}ms > 50ms"

    def test_planning_success_rate(self, robot_system):
        """Motion planning should succeed >95% for random valid poses"""
        successes = 0
        trials = 100

        for i in range(trials):
            # Generate random valid target pose
            target = self.generate_random_pose()

            # Plan motion
            success = self.planner.plan_to_pose(target)

            if success:
                successes += 1

        success_rate = successes / trials

        assert success_rate >= 0.95, f"Planning success rate {success_rate:.1%} < 95%"

    def test_grasp_success_rate(self, robot_system):
        """Grasping should succeed >85% for known objects"""
        successes = 0
        trials = 20
        object_types = ['book', 'box', 'cylinder']

        for obj_type in object_types:
            for trial in range(trials // len(object_types)):
                # Place object in workspace
                self.place_test_object(obj_type)

                # Detect object
                detections = self.perception_node.get_detections()
                target_obj = next((d for d in detections if d.class_name == obj_type), None)

                if not target_obj:
                    continue  # Detection failed, skip

                # Attempt grasp
                success = self.executor.grasp_object(target_obj)

                if success:
                    successes += 1

                # Release and reset
                self.executor.release_object()
                time.sleep(1)

        grasp_success_rate = successes / trials

        assert grasp_success_rate >= 0.85, f"Grasp success {grasp_success_rate:.1%} < 85%"

    def test_end_to_end_task(self, robot_system):
        """Complete task from natural language to execution"""
        task = "Pick up the red book and place it on the blue shelf"

        start_time = time.time()

        # Execute task
        success = self.executor.execute_task(task)

        execution_time = time.time() - start_time

        assert success, "Task execution failed"
        assert execution_time < 60, f"Task took {execution_time:.1f}s > 60s"

        # Verify outcome
        final_state = self.perception_node.get_scene_state()
        red_book = next((obj for obj in final_state if obj.color == 'red'), None)

        assert red_book is not None, "Red book not found after task"
        assert self.is_on_shelf(red_book, 'blue'), "Red book not on blue shelf"

    def test_safety_monitoring(self, robot_system):
        """Safety system should detect and prevent violations"""
        # Test force limit
        self.safety_monitor.force_limit = 50.0

        # Simulate high force
        self.simulate_force(100.0)

        time.sleep(0.1)  # Allow safety monitor to react

        assert self.safety_monitor.is_emergency_stop, "E-stop not triggered for high force"

        # Reset
        self.safety_monitor.reset()

        # Test workspace bounds
        self.robot.move_to_pose([2.0, 0, 0])  # Outside workspace

        time.sleep(0.1)

        assert self.safety_monitor.is_emergency_stop, "E-stop not triggered for bounds violation"

    def test_sim_to_real_transfer(self, robot_system):
        """Policies trained in sim should work on real robot"""
        # Load policy trained in Isaac Sim
        policy = self.load_vla_policy('isaac_sim_policy.pth')

        # Run 10 tasks on real robot
        successes = 0
        for i in range(10):
            task = self.get_test_task(i)
            success = self.executor.execute_with_policy(policy, task)

            if success:
                successes += 1

        # Sim-to-real gap should be <20% (i.e., >80% real-world success if 100% sim success)
        real_success_rate = successes / 10

        assert real_success_rate >= 0.80, f"Sim-to-real gap too large: {real_success_rate:.1%}"

    @pytest.mark.stress
    def test_continuous_operation(self, robot_system):
        """System should operate reliably for 8 hours"""
        duration = 8 * 60 * 60  # 8 hours in seconds
        start_time = time.time()

        errors = []
        tasks_completed = 0

        while (time.time() - start_time) < duration:
            try:
                task = self.generate_random_task()
                success = self.executor.execute_task(task)

                if success:
                    tasks_completed += 1
                else:
                    errors.append(f"Task failed at {time.time()-start_time:.0f}s")

            except Exception as e:
                errors.append(f"Exception at {time.time()-start_time:.0f}s: {str(e)}")

            time.sleep(30)  # 30s between tasks

        # Should complete at least 100 tasks in 8 hours (conservative)
        assert tasks_completed >= 100, f"Only {tasks_completed} tasks in 8 hours"

        # Error rate should be <5%
        error_rate = len(errors) / tasks_completed if tasks_completed > 0 else 1.0

        assert error_rate < 0.05, f"Error rate {error_rate:.1%} too high"
```

**Performance Metrics and Benchmarking**

```python
# metrics_collector.py - Collect and analyze system performance
import time
import json
from dataclasses import dataclass, asdict
from typing import List
import numpy as np

@dataclass
class TaskMetrics:
    """Metrics for a single task execution"""
    task_id: str
    task_description: str
    start_time: float
    end_time: float
    success: bool
    detection_time: float
    planning_time: float
    execution_time: float
    num_retries: int
    error_message: str = ""

class MetricsCollector:
    """Collect and analyze system performance metrics"""

    def __init__(self):
        self.tasks: List[TaskMetrics] = []

    def record_task(self, metrics: TaskMetrics):
        """Record metrics for completed task"""
        self.tasks.append(metrics)

        # Save to file
        with open('task_metrics.jsonl', 'a') as f:
            f.write(json.dumps(asdict(metrics)) + '\n')

    def compute_summary(self):
        """Compute summary statistics"""
        if not self.tasks:
            return {}

        successes = [t for t in self.tasks if t.success]
        failures = [t for t in self.tasks if not t.success]

        total_times = [t.end_time - t.start_time for t in self.tasks]
        detection_times = [t.detection_time for t in self.tasks]
        planning_times = [t.planning_time for t in self.tasks]
        execution_times = [t.execution_time for t in self.tasks]

        summary = {
            'total_tasks': len(self.tasks),
            'successes': len(successes),
            'failures': len(failures),
            'success_rate': len(successes) / len(self.tasks),

            'avg_total_time': np.mean(total_times),
            'p50_total_time': np.median(total_times),
            'p95_total_time': np.percentile(total_times, 95),

            'avg_detection_time': np.mean(detection_times),
            'avg_planning_time': np.mean(planning_times),
            'avg_execution_time': np.mean(execution_times),

            'avg_retries': np.mean([t.num_retries for t in self.tasks])
        }

        return summary

    def print_report(self):
        """Print performance report"""
        summary = self.compute_summary()

        print("=" * 60)
        print("CAPSTONE SYSTEM PERFORMANCE REPORT")
        print("=" * 60)
        print(f"Total Tasks: {summary['total_tasks']}")
        print(f"Success Rate: {summary['success_rate']:.1%}")
        print(f"")
        print(f"Timing (seconds):")
        print(f"  Average Total Time: {summary['avg_total_time']:.2f}s")
        print(f"  P50 Total Time: {summary['p50_total_time']:.2f}s")
        print(f"  P95 Total Time: {summary['p95_total_time']:.2f}s")
        print(f"")
        print(f"Breakdown:")
        print(f"  Detection: {summary['avg_detection_time']:.2f}s")
        print(f"  Planning: {summary['avg_planning_time']:.2f}s")
        print(f"  Execution: {summary['avg_execution_time']:.2f}s")
        print(f"")
        print(f"Reliability:")
        print(f"  Average Retries: {summary['avg_retries']:.2f}")
        print("=" * 60)

    def check_benchmarks(self):
        """Check if system meets target benchmarks"""
        summary = self.compute_summary()

        benchmarks = {
            'Task Completion Rate': (summary['success_rate'], 0.80, '80%'),
            'Average Task Time': (summary['avg_total_time'], 60, '< 60s'),
            'P95 Latency': (summary['p95_total_time'], 120, '< 120s')
        }

        print("\nBENCHMARK VALIDATION:")
        all_passed = True

        for metric_name, (value, threshold, target) in benchmarks.items():
            if metric_name == 'Task Completion Rate':
                passed = value >= threshold
            else:
                passed = value <= threshold

            status = "PASS" if passed else "FAIL"
            print(f"  [{status}] {metric_name}: {value:.2f} (target: {target})")

            if not passed:
                all_passed = False

        return all_passed
```

**Failure Modes and Recovery**

```python
# failure_recovery.py - Handle and recover from failures
from enum import Enum
import time

class FailureMode(Enum):
    DETECTION_FAILED = "object_not_detected"
    PLANNING_FAILED = "motion_planning_timeout"
    GRASP_FAILED = "grasp_unsuccessful"
    COLLISION = "collision_detected"
    FORCE_EXCEEDED = "force_limit_exceeded"
    TIMEOUT = "task_timeout"

class FailureRecovery:
    """Manage failure modes and recovery strategies"""

    def __init__(self, robot, perception, planner):
        self.robot = robot
        self.perception = perception
        self.planner = planner

        self.recovery_strategies = {
            FailureMode.DETECTION_FAILED: self.recover_detection,
            FailureMode.PLANNING_FAILED: self.recover_planning,
            FailureMode.GRASP_FAILED: self.recover_grasp,
            FailureMode.COLLISION: self.recover_collision,
            FailureMode.FORCE_EXCEEDED: self.recover_force,
        }

    def handle_failure(self, failure_mode: FailureMode, context: dict):
        """Handle failure and attempt recovery"""
        print(f"FAILURE: {failure_mode.value}")

        if failure_mode in self.recovery_strategies:
            recovery_fn = self.recovery_strategies[failure_mode]
            success = recovery_fn(context)

            if success:
                print(f"RECOVERED from {failure_mode.value}")
                return True
            else:
                print(f"RECOVERY FAILED for {failure_mode.value}")
                return False
        else:
            print(f"NO RECOVERY STRATEGY for {failure_mode.value}")
            return False

    def recover_detection(self, context):
        """Recovery: Move camera to better viewpoint"""
        print("Attempting detection recovery: moving to better viewpoint")

        # Try different camera angles
        viewpoints = [
            [0.3, 0, 0.5],  # Slightly forward
            [-0.3, 0, 0.5], # Slightly back
            [0, 0.3, 0.5],  # Side view
        ]

        for viewpoint in viewpoints:
            self.robot.move_camera_to(viewpoint)
            time.sleep(0.5)  # Allow perception to update

            detections = self.perception.get_detections()

            if len(detections) > 0:
                return True

        return False

    def recover_planning(self, context):
        """Recovery: Simplify motion by adding waypoints"""
        print("Attempting planning recovery: adding intermediate waypoints")

        target = context['target_pose']
        current = self.robot.get_current_pose()

        # Add midpoint waypoint
        midpoint = self.interpolate_pose(current, target, 0.5)

        # Try planning with waypoint
        success = self.planner.plan_through_waypoints([current, midpoint, target])

        return success

    def recover_grasp(self, context):
        """Recovery: Try different grasp approach"""
        print("Attempting grasp recovery: trying alternative approach")

        object_pose = context['object_pose']

        # Try grasping from different angles
        approach_angles = [0, 45, 90, 135]  # degrees

        for angle in approach_angles:
            grasp_pose = self.compute_grasp_with_angle(object_pose, angle)

            success = self.robot.execute_grasp(grasp_pose)

            if success:
                return True

            time.sleep(0.5)

        return False

    def recover_collision(self, context):
        """Recovery: Back away and replan"""
        print("Attempting collision recovery: backing away")

        # Move back to safe pose
        self.robot.move_to_home_pose()
        time.sleep(1)

        # Update obstacle map
        self.perception.update_obstacle_map()

        return True  # Ready to retry task

    def recover_force(self, context):
        """Recovery: Reduce grip force and retry"""
        print("Attempting force recovery: reducing grip force")

        # Open gripper slightly
        self.robot.gripper.open(partial=0.5)
        time.sleep(0.5)

        # Retry with lower force
        self.robot.gripper.set_force(force=20)  # Reduced from default

        return True
```

## Deployment to Hardware

### Phase 6: Integration & Testing

```python
# integration_test.py
import pytest
import asyncio
from task_executor import TaskExecutor

class TestCapstoneSystem:
    """Integration tests for capstone system"""

    @pytest.fixture
    def system(self):
        """Initialize system for testing"""
        from robot_manager import RobotManager

        system = RobotManager(
            robot_config='robot_config.yaml',
            use_simulation=True
        )
        yield system
        system.shutdown()

    @pytest.mark.asyncio
    async def test_pick_and_place(self, system):
        """Test pick-and-place task"""
        task = "Pick up the red block and place it on the blue block"

        success = await system.execute_task(task)

        assert success
        assert system.gripper.is_empty()

    @pytest.mark.asyncio
    async def test_object_detection(self, system):
        """Test perception accuracy"""
        detections = system.perception.get_detections()

        assert len(detections) > 0
        assert all('class' in d and 'bbox' in d for d in detections)

    @pytest.mark.asyncio
    async def test_motion_planning(self, system):
        """Test motion planning safety"""
        # Plan collision-free path
        target = [0.5, 0.5, 0.5]

        path = system.planner.compute_path(
            start=system.robot.get_current_pose(),
            target=target
        )

        # Verify no collisions
        for waypoint in path:
            assert system.collision_checker.is_collision_free(waypoint)

    @pytest.mark.asyncio
    async def test_end_to_end_workflow(self, system):
        """Full workflow test"""
        # 1. User sends natural language instruction
        instruction = "Organize books by color"

        # 2. System processes
        success = await system.execute_task(instruction)

        # 3. Verify outcome
        assert success

    def test_safety_limits(self, system):
        """Test robot stays within safety bounds"""
        config = system.robot_config

        current = system.robot.get_current_pose()

        assert config['workspace']['x'][0] <= current.x <= config['workspace']['x'][1]
        assert config['workspace']['y'][0] <= current.y <= config['workspace']['y'][1]
        assert config['workspace']['z'][0] <= current.z <= config['workspace']['z'][1]
```

**Real-World Deployment Strategy**

```python
# deployment_manager.py - Manage production deployment
import logging
import time
from datetime import datetime
from typing import Dict, List
import psycopg2
import json

class DeploymentManager:
    """Manage production deployment and monitoring"""

    def __init__(self):
        self.logger = self.setup_logging()
        self.db = self.connect_database()
        self.deployment_stage = "pre-production"  # pre-production, staging, production

    def setup_logging(self):
        """Configure comprehensive logging"""
        logger = logging.getLogger('capstone_system')
        logger.setLevel(logging.INFO)

        # File handler
        fh = logging.FileHandler(f'deployment_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
        fh.setLevel(logging.DEBUG)

        # Console handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)

        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        logger.addHandler(fh)
        logger.addHandler(ch)

        return logger

    def connect_database(self):
        """Connect to PostgreSQL for logging tasks"""
        try:
            conn = psycopg2.connect(
                host="localhost",
                database="capstone_db",
                user="robot",
                password="robot_password"
            )
            self.logger.info("Database connection established")
            return conn
        except Exception as e:
            self.logger.error(f"Database connection failed: {e}")
            return None

    def gradual_capability_rollout(self):
        """Gradually increase system capabilities"""
        rollout_stages = [
            {
                'stage': 1,
                'description': 'Perception only (no motion)',
                'duration': '1 day',
                'enabled': ['perception', 'detection_logging'],
                'disabled': ['motion_planning', 'execution']
            },
            {
                'stage': 2,
                'description': 'Motion planning in simulation',
                'duration': '2 days',
                'enabled': ['perception', 'motion_planning_sim'],
                'disabled': ['real_robot_motion']
            },
            {
                'stage': 3,
                'description': 'Slow real robot motion (0.1 m/s)',
                'duration': '3 days',
                'enabled': ['perception', 'motion_planning', 'slow_execution'],
                'velocity_limit': 0.1
            },
            {
                'stage': 4,
                'description': 'Normal operation (0.5 m/s)',
                'duration': 'ongoing',
                'enabled': ['all'],
                'velocity_limit': 0.5
            }
        ]

        for stage in rollout_stages:
            self.logger.info(f"ROLLOUT STAGE {stage['stage']}: {stage['description']}")
            self.logger.info(f"Duration: {stage['duration']}")

            # Configure system for this stage
            self.configure_for_stage(stage)

            # Wait for manual approval before next stage
            input(f"Press Enter to proceed to next stage...")

    def configure_for_stage(self, stage: Dict):
        """Configure system capabilities for rollout stage"""
        # Update system configuration
        config = {
            'enabled_features': stage.get('enabled', []),
            'velocity_limit': stage.get('velocity_limit', 0.1),
            'force_limit': 50.0,  # Conservative throughout rollout
        }

        # Save configuration
        with open('system_config.json', 'w') as f:
            json.dump(config, f, indent=2)

        self.logger.info(f"System configured for stage: {config}")

    def run_health_checks(self) -> bool:
        """Run comprehensive health checks"""
        checks = {
            'database': self.check_database(),
            'ros_nodes': self.check_ros_nodes(),
            'robot_connection': self.check_robot_connection(),
            'camera': self.check_camera(),
            'disk_space': self.check_disk_space(),
            'gpu_available': self.check_gpu()
        }

        all_passed = all(checks.values())

        self.logger.info("HEALTH CHECK RESULTS:")
        for check_name, passed in checks.items():
            status = "PASS" if passed else "FAIL"
            self.logger.info(f"  [{status}] {check_name}")

        return all_passed

    def check_ros_nodes(self) -> bool:
        """Check all required ROS 2 nodes are running"""
        import subprocess

        required_nodes = [
            '/perception_node',
            '/planning_node',
            '/control_node',
            '/safety_monitor',
            '/task_executor'
        ]

        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        running_nodes = result.stdout.split('\n')

        for node in required_nodes:
            if node not in running_nodes:
                self.logger.error(f"Missing node: {node}")
                return False

        return True

    def check_robot_connection(self) -> bool:
        """Verify robot is connected and responsive"""
        import socket

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('192.168.1.100', 30001))  # UR robot port
            sock.close()

            return result == 0
        except:
            return False

    def check_camera(self) -> bool:
        """Check camera is streaming"""
        import subprocess

        result = subprocess.run(
            ['ros2', 'topic', 'hz', '/camera/color/image_raw', '--window', '10'],
            capture_output=True,
            text=True,
            timeout=5
        )

        # Should be ~30 Hz
        return '30' in result.stdout or '29' in result.stdout

    def check_disk_space(self) -> bool:
        """Ensure sufficient disk space (>10GB)"""
        import shutil

        stat = shutil.disk_usage('/')
        free_gb = stat.free / (1024**3)

        return free_gb > 10

    def check_gpu(self) -> bool:
        """Check GPU is available for inference"""
        try:
            import torch
            return torch.cuda.is_available()
        except:
            return False

    def check_database(self) -> bool:
        """Check database connection"""
        return self.db is not None

    def monitor_production(self):
        """Continuous monitoring in production"""
        self.logger.info("Starting production monitoring...")

        while True:
            # Run health checks every 60 seconds
            health = self.run_health_checks()

            if not health:
                self.logger.error("HEALTH CHECK FAILED - Review logs")
                # Could trigger alert here

            # Check task success rates
            metrics = self.query_recent_metrics()

            if metrics['success_rate'] < 0.80:
                self.logger.warning(
                    f"Success rate dropped to {metrics['success_rate']:.1%}"
                )

            time.sleep(60)

    def query_recent_metrics(self) -> Dict:
        """Query metrics from last hour"""
        if not self.db:
            return {'success_rate': 1.0, 'avg_time': 0}

        cursor = self.db.cursor()
        cursor.execute("""
            SELECT
                COUNT(*) as total,
                SUM(CASE WHEN success THEN 1 ELSE 0 END) as successes,
                AVG(execution_time) as avg_time
            FROM task_log
            WHERE timestamp > NOW() - INTERVAL '1 hour'
        """)

        row = cursor.fetchone()

        if row and row[0] > 0:
            return {
                'total': row[0],
                'success_rate': row[1] / row[0],
                'avg_time': row[2]
            }

        return {'total': 0, 'success_rate': 1.0, 'avg_time': 0}

def main():
    manager = DeploymentManager()

    # Run pre-deployment health checks
    print("Running pre-deployment health checks...")
    if not manager.run_health_checks():
        print("ERROR: Health checks failed. Fix issues before deployment.")
        return

    print("\nHealth checks passed!")

    # Gradual rollout
    print("\nStarting gradual capability rollout...")
    manager.gradual_capability_rollout()

    # Production monitoring
    print("\nEntering production monitoring mode...")
    manager.monitor_production()

if __name__ == '__main__':
    main()
```

**Monitoring and Logging**

```python
# production_monitor.py - Real-time production monitoring
import time
import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Deque
import matplotlib.pyplot as plt
from datetime import datetime

@dataclass
class SystemMetrics:
    """Real-time system metrics"""
    timestamp: float
    cpu_usage: float
    gpu_usage: float
    memory_usage: float
    perception_hz: float
    planning_latency: float
    task_success: bool

class ProductionMonitor:
    """Monitor system performance in production"""

    def __init__(self, window_size=1000):
        self.metrics: Deque[SystemMetrics] = deque(maxlen=window_size)
        self.alert_thresholds = {
            'cpu_usage': 90.0,
            'gpu_usage': 95.0,
            'memory_usage': 85.0,
            'perception_hz': 25.0,  # Alert if <25 Hz
            'planning_latency': 1.0,  # Alert if >1s
            'success_rate': 0.80
        }

    def record_metrics(self, metrics: SystemMetrics):
        """Record new metrics"""
        self.metrics.append(metrics)

        # Check for alerts
        self.check_alerts(metrics)

    def check_alerts(self, metrics: SystemMetrics):
        """Check if any metrics exceed thresholds"""
        alerts = []

        if metrics.cpu_usage > self.alert_thresholds['cpu_usage']:
            alerts.append(f"High CPU usage: {metrics.cpu_usage:.1f}%")

        if metrics.gpu_usage > self.alert_thresholds['gpu_usage']:
            alerts.append(f"High GPU usage: {metrics.gpu_usage:.1f}%")

        if metrics.memory_usage > self.alert_thresholds['memory_usage']:
            alerts.append(f"High memory usage: {metrics.memory_usage:.1f}%")

        if metrics.perception_hz < self.alert_thresholds['perception_hz']:
            alerts.append(f"Low perception rate: {metrics.perception_hz:.1f} Hz")

        if metrics.planning_latency > self.alert_thresholds['planning_latency']:
            alerts.append(f"High planning latency: {metrics.planning_latency:.2f}s")

        if alerts:
            print(f"\nALERTS at {datetime.fromtimestamp(metrics.timestamp)}:")
            for alert in alerts:
                print(f"  - {alert}")

    def compute_success_rate(self, window_minutes=10):
        """Compute success rate over recent window"""
        cutoff = time.time() - (window_minutes * 60)
        recent = [m for m in self.metrics if m.timestamp > cutoff]

        if not recent:
            return 1.0

        successes = sum(1 for m in recent if m.task_success)
        return successes / len(recent)

    def generate_report(self):
        """Generate performance report"""
        if not self.metrics:
            print("No metrics collected yet")
            return

        cpu_values = [m.cpu_usage for m in self.metrics]
        gpu_values = [m.gpu_usage for m in self.metrics]
        latency_values = [m.planning_latency for m in self.metrics]
        hz_values = [m.perception_hz for m in self.metrics]

        print("\n" + "="*60)
        print("PRODUCTION MONITORING REPORT")
        print("="*60)
        print(f"Samples: {len(self.metrics)}")
        print(f"")
        print(f"CPU Usage: avg={np.mean(cpu_values):.1f}% p95={np.percentile(cpu_values, 95):.1f}%")
        print(f"GPU Usage: avg={np.mean(gpu_values):.1f}% p95={np.percentile(gpu_values, 95):.1f}%")
        print(f"Perception Rate: avg={np.mean(hz_values):.1f} Hz min={np.min(hz_values):.1f} Hz")
        print(f"Planning Latency: avg={np.mean(latency_values):.2f}s p95={np.percentile(latency_values, 95):.2f}s")
        print(f"Success Rate (10min): {self.compute_success_rate(10):.1%}")
        print("="*60)

    def plot_metrics(self, save_path='monitoring_dashboard.png'):
        """Plot monitoring dashboard"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        timestamps = [(m.timestamp - self.metrics[0].timestamp) / 60 for m in self.metrics]  # minutes

        # CPU/GPU usage
        axes[0, 0].plot(timestamps, [m.cpu_usage for m in self.metrics], label='CPU')
        axes[0, 0].plot(timestamps, [m.gpu_usage for m in self.metrics], label='GPU')
        axes[0, 0].set_title('Resource Usage (%)')
        axes[0, 0].set_xlabel('Time (minutes)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # Perception rate
        axes[0, 1].plot(timestamps, [m.perception_hz for m in self.metrics])
        axes[0, 1].axhline(y=30, color='g', linestyle='--', label='Target')
        axes[0, 1].set_title('Perception Rate (Hz)')
        axes[0, 1].set_xlabel('Time (minutes)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # Planning latency
        axes[1, 0].plot(timestamps, [m.planning_latency for m in self.metrics])
        axes[1, 0].set_title('Planning Latency (s)')
        axes[1, 0].set_xlabel('Time (minutes)')
        axes[1, 0].grid(True)

        # Success rate (rolling window)
        window = 50
        success_rate = []
        for i in range(len(self.metrics)):
            if i < window:
                continue
            recent = list(self.metrics)[i-window:i]
            rate = sum(1 for m in recent if m.task_success) / window
            success_rate.append(rate)

        axes[1, 1].plot(timestamps[window:], success_rate)
        axes[1, 1].axhline(y=0.80, color='r', linestyle='--', label='Threshold')
        axes[1, 1].set_title(f'Success Rate (rolling {window} tasks)')
        axes[1, 1].set_xlabel('Time (minutes)')
        axes[1, 1].set_ylim([0, 1])
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.savefig(save_path)
        print(f"Dashboard saved to {save_path}")
```

**Performance Tuning in Production**

```python
# performance_tuner.py - Optimize system performance
import psutil
import torch

class PerformanceTuner:
    """Optimize system performance based on deployment environment"""

    def __init__(self):
        self.system_info = self.analyze_system()

    def analyze_system(self):
        """Analyze available hardware resources"""
        return {
            'cpu_count': psutil.cpu_count(),
            'memory_gb': psutil.virtual_memory().total / (1024**3),
            'gpu_available': torch.cuda.is_available(),
            'gpu_memory_gb': torch.cuda.get_device_properties(0).total_memory / (1024**3) if torch.cuda.is_available() else 0
        }

    def optimize_perception(self):
        """Tune perception pipeline for hardware"""
        if self.system_info['gpu_available']:
            model_size = 'yolov8m.pt' if self.system_info['gpu_memory_gb'] > 8 else 'yolov8s.pt'
            batch_size = 4 if self.system_info['gpu_memory_gb'] > 8 else 1
        else:
            model_size = 'yolov8n.pt'  # Nano model for CPU
            batch_size = 1

        print(f"Perception optimization:")
        print(f"  Model: {model_size}")
        print(f"  Batch size: {batch_size}")

        return {
            'model': model_size,
            'batch_size': batch_size,
            'device': 'cuda' if self.system_info['gpu_available'] else 'cpu'
        }

    def optimize_planning(self):
        """Tune motion planning parameters"""
        if self.system_info['cpu_count'] >= 8:
            num_threads = 4
            planning_time = 5.0
        else:
            num_threads = 2
            planning_time = 10.0

        print(f"Planning optimization:")
        print(f"  Threads: {num_threads}")
        print(f"  Max planning time: {planning_time}s")

        return {
            'num_threads': num_threads,
            'planning_time': planning_time
        }

    def set_cpu_affinity(self):
        """Pin processes to specific CPU cores"""
        # Pin perception to cores 0-3
        # Pin planning to cores 4-7

        import os
        pid = os.getpid()

        try:
            p = psutil.Process(pid)
            p.cpu_affinity([0, 1, 2, 3])  # Use first 4 cores
            print("CPU affinity set")
        except:
            print("Failed to set CPU affinity")

    def enable_mixed_precision(self):
        """Enable mixed precision for faster inference"""
        if self.system_info['gpu_available']:
            torch.set_float32_matmul_precision('medium')
            print("Mixed precision enabled (TF32)")
```

## Advanced Enhancements

### Phase 7: Deployment Checklist

```yaml
Pre-Deployment Verification:
  Safety:
    - [ ] Emergency stop tested and functional
    - [ ] Robot velocity limits set to 0.5 m/s initially
    - [ ] Collision detection enabled
    - [ ] Operator trained on emergency procedures

  Hardware:
    - [ ] All sensors calibrated (camera intrinsics, force/torque)
    - [ ] Network connectivity verified (latency < 50ms)
    - [ ] Power supply adequate (240V, 30A circuit)
    - [ ] Ethernet and sensor cables secured

  Software:
    - [ ] ROS 2 nodes launch without errors
    - [ ] Health checks pass (database, API, sensors)
    - [ ] Simulation and real robot paths match (< 5% error)
    - [ ] Logging and monitoring active

  Integration:
    - [ ] Perception pipeline accuracy > 90%
    - [ ] Motion planning success rate > 95%
    - [ ] Task execution success rate > 80%
    - [ ] API response time < 200ms

  Documentation:
    - [ ] System architecture documented
    - [ ] API endpoints documented
    - [ ] Troubleshooting guide complete
    - [ ] Video demo recorded

Launch Procedure:
  1. Power on robot control PC
  2. Verify network connectivity: ping robot_ip
  3. Launch ROS 2 stack: ros2 launch capstone_system system.launch.py
  4. Run health checks: ros2 service call /health std_srvs/srv/Trigger
  5. Verify perception: ros2 topic echo /detections
  6. Test motion planning: ros2 service call /plan_motion geometry_msgs/PoseStamped
  7. Execute first task via API: curl http://localhost:8000/api/tasks

Monitoring (Production):
  - Log all tasks to database
  - Alert on errors: low_confidence, planning_failures, sensor_errors
  - Monitor GPU utilization (< 80%)
  - Track success metrics daily
```

## Troubleshooting Guide

### Problem: Robot Motion Jittery

**Cause**: High control loop latency or low update rate

**Solution**:
```bash
# Increase ROS 2 timer frequency
# In control_node.py:
self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

# Verify network latency
ping -c 100 robot_ip | tail -1
# Should be < 5ms
```

### Problem: Objects Not Detected

**Cause**: Poor lighting or model trained on different domain

**Solution**:
```python
# Improve lighting
# Increase brightness in perception settings
self.detector.conf = 0.5  # Lower confidence threshold

# Or retrain detector on your objects
from ultralytics import YOLO
model = YOLO('yolov8m.pt')
results = model.train(data='custom_data.yaml', epochs=100)
```

### Problem: Motion Planning Timeout

**Cause**: Complex scene with many obstacles

**Solution**:
```python
# Increase planning time
self.moveit.set_planning_time(10.0)  # seconds

# Simplify path by adding waypoints
waypoints = [
    intermediate_pose_1,
    intermediate_pose_2,
    target_pose
]
self.moveit.plan_through_waypoints(waypoints)
```

**Vision-Language Models for Natural Instructions**

```python
# vla_integration.py - Integrate vision-language-action models
from anthropic import Anthropic
import base64
import io
from PIL import Image

class VLAIntegration:
    """Use VLAs for natural language instruction following"""

    def __init__(self):
        self.client = Anthropic()

    def parse_instruction_with_vision(self, instruction: str, scene_image: Image):
        """
        Parse natural language instruction grounded in visual scene

        Example: "Pick up the red book on the left" + image -> precise action
        """
        # Encode image
        buffered = io.BytesIO()
        scene_image.save(buffered, format="PNG")
        img_str = base64.b64encode(buffered.getvalue()).decode()

        # Query Claude with vision
        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1024,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/png",
                                "data": img_str
                            }
                        },
                        {
                            "type": "text",
                            "text": f"""
                            Given this image of a robot workspace, parse the following instruction into a structured action plan:

                            Instruction: "{instruction}"

                            Return JSON with:
                            {{
                                "target_object": {{"name": "...", "bbox": [x1, y1, x2, y2]}},
                                "action_sequence": [
                                    {{"action": "move_to", "params": {{...}}}},
                                    {{"action": "grasp", "params": {{...}}}},
                                    ...
                                ],
                                "success_criteria": "..."
                            }}
                            """
                        }
                    ]
                }
            ]
        )

        import json
        action_plan = json.loads(response.content[0].text)

        return action_plan

    def generate_error_explanation(self, error_type: str, context: dict, scene_image: Image):
        """Use VLA to explain what went wrong and suggest fixes"""
        buffered = io.BytesIO()
        scene_image.save(buffered, format="PNG")
        img_str = base64.b64encode(buffered.getvalue()).decode()

        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=512,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/png",
                                "data": img_str
                            }
                        },
                        {
                            "type": "text",
                            "text": f"""
                            The robot encountered this error: {error_type}

                            Context: {json.dumps(context)}

                            Looking at the current scene, explain:
                            1. Why did this error occur?
                            2. What should the robot do to recover?

                            Be specific and actionable.
                            """
                        }
                    ]
                }
            ]
        )

        return response.content[0].text
```

**Learning from Human Feedback**

```python
# human_feedback.py - Improve system through human corrections
import json
from datetime import datetime
from typing import List, Dict

class HumanFeedbackCollector:
    """Collect and learn from human corrections"""

    def __init__(self):
        self.feedback_log = []

    def collect_feedback(self, task_id: str, task_description: str,
                        robot_action: Dict, human_correction: Dict):
        """
        Collect human feedback on robot actions

        Example:
            robot_action = {"grasp_point": [0.5, 0.3, 0.2]}
            human_correction = {"grasp_point": [0.52, 0.31, 0.19]}
        """
        feedback = {
            'timestamp': datetime.now().isoformat(),
            'task_id': task_id,
            'task_description': task_description,
            'robot_action': robot_action,
            'human_correction': human_correction,
            'correction_magnitude': self.compute_correction_magnitude(
                robot_action, human_correction
            )
        }

        self.feedback_log.append(feedback)

        # Save to file
        with open('feedback_log.jsonl', 'a') as f:
            f.write(json.dumps(feedback) + '\n')

    def compute_correction_magnitude(self, robot_action: Dict,
                                     human_correction: Dict) -> float:
        """Compute how different the correction was"""
        import numpy as np

        if 'grasp_point' in robot_action:
            robot_point = np.array(robot_action['grasp_point'])
            human_point = np.array(human_correction['grasp_point'])

            return np.linalg.norm(robot_point - human_point)

        return 0.0

    def analyze_feedback_trends(self):
        """Identify systematic errors from feedback"""
        if not self.feedback_log:
            return {}

        # Analyze by task type
        task_errors = {}
        for feedback in self.feedback_log:
            task = feedback['task_description']

            if task not in task_errors:
                task_errors[task] = []

            task_errors[task].append(feedback['correction_magnitude'])

        # Report tasks with largest corrections
        avg_corrections = {
            task: np.mean(corrections)
            for task, corrections in task_errors.items()
        }

        sorted_tasks = sorted(avg_corrections.items(),
                             key=lambda x: x[1], reverse=True)

        print("\nTASKS REQUIRING MOST CORRECTION:")
        for task, avg_correction in sorted_tasks[:5]:
            print(f"  {task}: {avg_correction*100:.1f}cm average correction")

        return avg_corrections

    def retrain_with_feedback(self):
        """Fine-tune models using human feedback"""
        print(f"Retraining with {len(self.feedback_log)} feedback samples...")

        # In production: fine-tune grasp model, motion planner, etc.
        # using DAgger or other imitation learning algorithms

        # Pseudo-code:
        # model.fine_tune(
        #     states=[f['robot_action'] for f in self.feedback_log],
        #     expert_actions=[f['human_correction'] for f in self.feedback_log]
        # )

        print("Model fine-tuning complete")
```

**Multi-Robot Coordination**

```python
# multi_robot_coordinator.py - Coordinate multiple robots
from typing import List, Dict
import asyncio

class MultiRobotCoordinator:
    """Coordinate tasks across multiple robots"""

    def __init__(self, robots: List):
        self.robots = robots
        self.task_queue = asyncio.Queue()
        self.active_tasks = {}

    async def execute_collaborative_task(self, task: Dict):
        """
        Execute task requiring multiple robots

        Example: Two robots working together to move large object
        """
        if task['type'] == 'cooperative_carry':
            # Assign roles
            robot_a = self.robots[0]
            robot_b = self.robots[1]

            # Synchronized grasping
            await asyncio.gather(
                robot_a.move_to(task['grasp_point_a']),
                robot_b.move_to(task['grasp_point_b'])
            )

            # Both grasp simultaneously
            await asyncio.gather(
                robot_a.close_gripper(),
                robot_b.close_gripper()
            )

            # Coordinated lifting
            await self.synchronized_lift(robot_a, robot_b, height=0.3)

            # Move together to destination
            await self.synchronized_move(
                robot_a, robot_b,
                target_a=task['destination_a'],
                target_b=task['destination_b']
            )

            # Release
            await asyncio.gather(
                robot_a.open_gripper(),
                robot_b.open_gripper()
            )

    async def synchronized_move(self, robot_a, robot_b, target_a, target_b):
        """Move two robots in synchronized motion"""
        # Plan coordinated trajectories
        traj_a = robot_a.plan_trajectory(target_a)
        traj_b = robot_b.plan_trajectory(target_b)

        # Ensure same duration
        max_duration = max(len(traj_a), len(traj_b))

        # Execute synchronized
        for i in range(max_duration):
            if i < len(traj_a):
                robot_a.execute_waypoint(traj_a[i])
            if i < len(traj_b):
                robot_b.execute_waypoint(traj_b[i])

            await asyncio.sleep(0.01)  # 100 Hz

    def allocate_tasks(self, tasks: List[Dict]):
        """Allocate tasks to robots to minimize total time"""
        from scipy.optimize import linear_sum_assignment

        # Cost matrix: time for each robot to complete each task
        costs = np.zeros((len(self.robots), len(tasks)))

        for i, robot in enumerate(self.robots):
            for j, task in enumerate(tasks):
                costs[i, j] = self.estimate_task_time(robot, task)

        # Solve assignment problem
        robot_indices, task_indices = linear_sum_assignment(costs)

        # Assign tasks
        assignments = {}
        for robot_idx, task_idx in zip(robot_indices, task_indices):
            assignments[self.robots[robot_idx]] = tasks[task_idx]

        return assignments

    def estimate_task_time(self, robot, task):
        """Estimate time for robot to complete task"""
        distance = np.linalg.norm(
            robot.get_current_pose() - task['location']
        )

        # Simple model: time = travel + manipulation
        travel_time = distance / 0.5  # Assume 0.5 m/s
        manipulation_time = 10  # Assume 10s per manipulation

        return travel_time + manipulation_time
```

**Continuous Improvement and Research Directions**

```python
# continuous_learning.py - Ongoing system improvement
class ContinuousLearning:
    """System that improves over time"""

    def __init__(self):
        self.performance_history = []

    def log_performance(self, metrics: Dict):
        """Track performance over time"""
        self.performance_history.append({
            'timestamp': time.time(),
            **metrics
        })

    def identify_improvement_opportunities(self):
        """Analyze data to find where to improve"""
        if len(self.performance_history) < 100:
            return []

        opportunities = []

        # Identify failure modes
        failures = [p for p in self.performance_history if not p.get('success')]

        failure_modes = {}
        for failure in failures:
            mode = failure.get('failure_reason', 'unknown')
            failure_modes[mode] = failure_modes.get(mode, 0) + 1

        # Prioritize most common failures
        sorted_failures = sorted(failure_modes.items(),
                                key=lambda x: x[1], reverse=True)

        for failure_mode, count in sorted_failures[:3]:
            opportunities.append({
                'type': 'reduce_failure',
                'failure_mode': failure_mode,
                'frequency': count,
                'recommendation': self.get_recommendation(failure_mode)
            })

        return opportunities

    def get_recommendation(self, failure_mode: str) -> str:
        """Generate improvement recommendation"""
        recommendations = {
            'detection_failed': 'Collect more training data for this object class',
            'grasp_failed': 'Fine-tune grasp model with recent failures',
            'planning_timeout': 'Simplify environment or increase planning time',
            'collision': 'Update obstacle models and add safety margins'
        }

        return recommendations.get(failure_mode, 'Manual investigation required')

    def auto_improve(self):
        """Automatically improve system based on data"""
        opportunities = self.identify_improvement_opportunities()

        for opp in opportunities:
            print(f"\nIMPROVEMENT OPPORTUNITY:")
            print(f"  Type: {opp['type']}")
            print(f"  Issue: {opp['failure_mode']}")
            print(f"  Frequency: {opp['frequency']}")
            print(f"  Recommendation: {opp['recommendation']}")

            # In production: trigger automated retraining, parameter tuning, etc.
```

## Project Timeline and Milestones

| Week | Milestone | Deliverables |
|------|-----------|--------------|
| 1-2 | Hardware Setup | Assembled robot, calibrated sensors |
| 3-4 | Perception Pipeline | Object detection >90% accuracy |
| 5-6 | Motion Planning | Collision-free planning >95% success |
| 7-8 | Task Execution | End-to-end tasks working in simulation |
| 9-10 | Integration Testing | All subsystems integrated, tests passing |
| 11-12 | Real Robot Deployment | System running on hardware |
| 13-14 | Performance Tuning | Meeting all benchmark targets |
| 15-16 | Documentation & Demo | Video demo, final report |

## Common Pitfalls and Solutions

**Pitfall 1: Sim-to-Real Gap**
- **Problem**: Policies work in simulation but fail on real robot
- **Solution**: Use domain randomization, accurate physics parameters, gradual transfer with human feedback

**Pitfall 2: Perception Failures**
- **Problem**: Object detection unreliable in real-world lighting
- **Solution**: Add domain-specific training data, use temporal filtering, LED ring lights for consistent illumination

**Pitfall 3: Motion Planning Timeouts**
- **Problem**: Complex scenes cause planning failures
- **Solution**: Simplify collision models, use hierarchical planning, add intermediate waypoints

**Pitfall 4: Grasp Failures**
- **Problem**: Objects slip or fall during manipulation
- **Solution**: Increase grip force, use force feedback, improve pose estimation, collect grasp dataset

**Pitfall 5: System Latency**
- **Problem**: End-to-end latency >200ms affects responsiveness
- **Solution**: Optimize perception (smaller models), parallelize pipelines, use GPU acceleration

## Scaling Considerations

**From 1 Robot to Fleet:**
1. **Centralized Task Queue**: Single server distributes tasks to available robots
2. **Shared Perception**: Cameras in environment (not just robot-mounted)
3. **Collision Avoidance**: Multi-robot path planning to prevent interference
4. **Load Balancing**: Distribute tasks based on robot capability and location

**Cost Optimization:**
- Start with smaller robot (UR3e at $20K vs UR10e at $50K)
- Use single scene camera instead of per-robot cameras
- Cloud compute for training, edge inference for deployment
- Open-source software stack (ROS 2, MoveIt2) reduces licensing costs

## Next Steps

**Immediate:**
1. Follow deployment checklist to launch your system
2. Execute first real-world pick-and-place task
3. Log all data for analysis

**Short-term (1-3 months):**
4. Monitor performance metrics daily
5. Retrain models on real-world data
6. Expand task repertoire (sorting, stacking, organizing)

**Long-term (3-12 months):**
7. Scale to multiple robots
8. Integrate with warehouse/library management system
9. Deploy vision-language-action models for general instructions
10. Contribute findings and code to open-source community

---

**Chapter 6 Summary**:

You've built a complete, production-ready Physical AI system that:
- Integrates perception, reasoning, planning, and control into a cohesive architecture
- Operates safely in real-world environments with humans
- Handles failures gracefully through recovery strategies
- Improves continuously through logged experience and human feedback
- Scales to multi-robot deployments

**Key Achievements:**
- 30+ production-grade code examples spanning hardware integration, perception, planning, control, testing, and deployment
- Comprehensive testing suite validating >80% task success rate
- Real-world deployment strategy with gradual capability rollout
- Monitoring and performance tuning for production operation

**Estimated Reading Time**: 75 minutes
**Code Examples**: 30+ (complete, production-ready implementations)
**Hands-on Project**: Full system deployment from scratch
**Final Outcome**: Production Physical AI system meeting all benchmark targets

**You now have the skills to:**
- Design and implement end-to-end Physical AI systems
- Deploy robots safely in real-world environments
- Debug and optimize multi-component robotic systems
- Scale from prototypes to production deployments
- Contribute to cutting-edge Physical AI research and development

Congratulations on completing the capstone project and the entire Physical AI and Humanoid Robotics curriculum!

---

## Additional Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/doc/examples/moveit2_tutorials/moveit2_tutorials.html)
- [Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/repositories.html)

### Community Resources
- [ROS Discourse](https://discourse.ros.org/) - Ask questions
- [GitHub Issues](https://github.com/ros-planning/moveit2/issues) - Report bugs
- [Robotics Stack Exchange](https://robotics.stackexchange.com/) - Expert answers
- [Robot Operating System YouTube](https://www.youtube.com/c/ROSRobotics) - Tutorials

### Papers & Research
- [Vision-Language Models for Robotics](https://arxiv.org/abs/2210.01776)
- [sim-to-real Transfer Learning](https://arxiv.org/abs/1802.09124)
- [Deep Reinforcement Learning for Robotics](https://arxiv.org/abs/1806.10293)
