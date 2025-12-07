# Chapter 6: Capstone Project - End-to-End Physical AI System

## Overview

This capstone project brings together all concepts from Chapters 1-5 to build a complete, production-ready Physical AI system. You'll implement a **robotic book assistant** that:

1. **Perceives**: Uses computer vision to detect objects and read text
2. **Reasons**: Understands natural language instructions via language models
3. **Acts**: Controls a robot arm to manipulate books and objects
4. **Learns**: Improves through experience and user feedback

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

## Next Steps

- **Deploy**: Follow deployment checklist to launch system
- **Iterate**: Monitor task success and retrain models on failures
- **Scale**: Add more robots and tasks
- **Share**: Contribute improvements back to open-source community

---

**Chapter 6 Summary**:
- Complete Physical AI system integrates perception, planning, and control
- Real-world deployment requires careful safety verification
- Continuous monitoring and iteration essential for reliability
- Architecture supports scaling to multi-robot scenarios

**Estimated Reading Time**: 35 minutes
**Code Examples**: 8 (configuration, perception, motion control, task execution, integration tests, deployment)
**Hands-on**: Deploy system and execute first real-world task
**Completion**: You now have skills to build production Physical AI systems!

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
