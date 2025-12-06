# Chapter 3: Gazebo Simulation

## Introduction to Gazebo

Gazebo is the most widely used robotics simulator in ROS 2. It provides:

- **Physics Simulation**: Accurate dynamics for rigid bodies
- **Sensor Simulation**: Camera, LiDAR, IMU, force sensors
- **Multi-Robot Support**: Simulate multiple robots simultaneously
- **Realistic Rendering**: OpenGL graphics for visual testing
- **Plugin System**: Extend functionality with custom controllers

### Why Simulate?

1. **Safety**: Test dangerous scenarios without risk
2. **Cost**: Avoid expensive hardware damage
3. **Speed**: Faster iteration than real hardware
4. **Repeatability**: Exact same conditions every test
5. **Development**: Develop software before hardware arrives

## Installation

```bash
# Ubuntu 22.04
sudo apt install ros-humble-gazebo-*

# Verify installation
gazebo --version  # Should show version
rviz2 &
```

## Basic Concepts

### World
Container for all simulation objects, physics, and settings.

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics settings -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <light name="sun">
      <pose>0 0 10 0 0 0</pose>
      <intensity>1.0</intensity>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <link name="link">
        <collision>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Your robot model here -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Models
Reusable objects (robots, objects, buildings).

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="my_robot">
    <!-- Links with geometry and physics -->
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
      </inertial>
      <collision>
        <geometry>
          <box><size>0.3 0.3 0.2</size></box>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <box><size>0.3 0.3 0.2</size></box>
        </geometry>
      </visual>
    </link>

    <!-- Joints connecting links -->
    <joint name="wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_link</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>
  </model>
</sdf>
```

## Plugins for ROS 2

### Differential Drive Plugin

```xml
<plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive">
  <ros>
    <namespace>/robot</namespace>
    <argument>cmd_vel:=cmd_vel</argument>
    <argument>odom:=odom</argument>
  </ros>

  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>

  <wheel_separation>0.5</wheel_separation>
  <wheels_per_side>1</wheels_per_side>
  <wheel_radius>0.2</wheel_radius>
</plugin>
```

### Camera Sensor Plugin

```xml
<sensor name="camera" type="camera">
  <pose>0 0 0.1 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
    <ros>
      <namespace>/robot</namespace>
      <argument>camera_name:=camera</argument>
    </ros>
  </plugin>
</sensor>
```

## Running Simulations

### Launch File

Create `launch/gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', 'world.sdf'],
        output='screen'
    )

    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['robot.sdf', '-x', '0', '-y', '0', '-z', '0']
    )

    return LaunchDescription([
        gazebo,
        robot_spawner
    ])
```

### Launch the Simulation

```bash
# Launch Gazebo with your world
ros2 launch my_robot_pkg gazebo.launch.py

# In another terminal, test your robot
ros2 topic pub /robot/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}"
```

## Common Simulation Tasks

### Adjusting Physics

```xml
<!-- Change gravity -->
<physics type="ode">
  <gravity>0 0 -9.81</gravity>
  <max_step_size>0.01</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### Adding Friction

```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>
      <mu2>0.8</mu2>
    </ode>
  </friction>
</surface>
```

### Testing Sensors

```python
# Subscribe to simulated sensor data
def camera_callback(msg):
    print(f"Image received: {msg.width}x{msg.height}")

node.create_subscription(Image, '/robot/camera/image_raw', camera_callback, 10)
```

## Best Practices

1. **Model Organization**
   - Use standard models from Gazebo library
   - Keep custom models simple initially
   - Version control your SDF files

2. **Physics Tuning**
   - Start with realistic gravity
   - Adjust step size for accuracy vs. speed
   - Test with actual robot hardware

3. **Debugging**
   - Use Gazebo GUI to visualize collisions
   - Enable logging for sensor data
   - Compare sim vs. hardware performance

## Troubleshooting

**Simulation Too Slow**
- Reduce number of simulation steps
- Simplify collision meshes
- Lower graphics quality

**Unstable Physics**
- Increase max_step_size
- Adjust friction coefficients
- Use smaller objects initially

**Sensor Not Publishing**
- Verify plugin is loaded
- Check ROS topic names
- Confirm sensor is in world

## Next Steps

- Build and simulate your robot
- Test perception algorithms in Gazebo
- Validate control algorithms before hardware
- **Next Chapter**: NVIDIA Isaac Platform

---

**Chapter 3 Summary**:
- Gazebo provides accurate physics and sensor simulation
- SDF format describes worlds, models, and sensors
- ROS 2 plugins integrate simulation with your code
- Test everything in simulation before hardware

**Estimated Reading Time**: 20 minutes
**Code Examples**: 6 (world, model, plugins, launch, physics)
**Hands-on**: Simulate a differential drive robot
**Next Chapter**: NVIDIA Isaac Platform
