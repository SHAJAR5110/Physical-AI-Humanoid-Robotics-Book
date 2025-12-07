# Chapter 3: Gazebo Simulation

## Introduction to Gazebo

Gazebo is the most widely used open-source robotics simulator in the ROS ecosystem. It bridges the gap between software development and real hardware by providing a physics-accurate virtual environment where robots can learn, test, and train without risk or cost.

**Core Capabilities**:
- **Physics Simulation**: ODE, Bullet, DART engines for rigid body dynamics
- **Sensor Simulation**: Cameras, LiDAR, IMU, force/torque, depth sensors
- **Multi-Robot Support**: Simulate multiple robots interacting simultaneously
- **Realistic Rendering**: OpenGL/OSG graphics for visual fidelity testing
- **Plugin System**: Extend with custom controllers and behaviors
- **Integration**: Seamless connection with ROS 2 middleware
- **Cloud Support**: Gazebo as a service in AWS RoboMaker

**Why Simulate Before Deploying to Hardware?**

**1. Safety**:
- Test dangerous scenarios without risk (collision, falls, system failures)
- Validate emergency stop procedures
- Verify fail-safe behaviors
- Simulate edge cases that are unsafe to test on real hardware
- Train in hazardous environments (high voltage, extreme temperatures)

**2. Cost Reduction**:
- Hardware costs: Robot arms ($50k-$500k), mobile bases ($10k-$100k), sensors ($500-$10k each)
- Avoid expensive hardware damage during development
- Test wear-and-tear scenarios without consuming hardware lifespan
- Develop algorithms before hardware procurement
- Example: Tesla Optimus development estimated at $25B; simulation saved costs by 30-40%

**3. Speed and Iteration**:
- Real-world test: 1 run every 30 seconds (setup, deploy, reset)
- Simulation: 100 runs per second (or faster in accelerated mode)
- Algorithm development: 10-50x faster iteration cycle
- Parallel testing: Run multiple experiments simultaneously
- Overnight training: Train controllers while sleeping

**4. Repeatability**:
- Exact same conditions every test (gravity, friction, lighting)
- Eliminate hardware variability (wear, temperature, sensor noise)
- Reproducible results for academic publishing
- Version control your scenarios
- Easy regression testing for software updates

**5. Development Parallelization**:
- Develop software before hardware arrives
- Multiple teams work on same robot simultaneously
- Integration testing before real-world deployment
- Continuous integration pipelines with simulation
- Example: Boston Dynamics runs millions of simulation hours daily

**Simulation-to-Reality Transfer**:
- Domain randomization: randomize physics parameters during training
- Sim2Real gap: typically 10-15% performance drop
- Transfer learning: models trained in sim transfer to real robots
- Safety validation: verify safety constraints before deployment

## Installation

### Ubuntu 22.04 (Recommended)

Gazebo is officially integrated with ROS 2 Humble:

```bash
# Step 1: Install Gazebo packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# Step 2: Install optional simulation packages
sudo apt install ros-humble-gazebo-plugins  # Sensors, drive plugins
sudo apt install ros-humble-gazebo-msgs     # ROS 2 message types

# Step 3: Install additional tools
sudo apt install ros-humble-gazebo-classic  # Legacy Gazebo 11 support
sudo apt install libgazebo-dev              # Development headers

# Step 4: Verify installation
gazebo --version         # Should show Gazebo version
ros2 pkg list | grep gazebo  # List all Gazebo packages

# Step 5: Test with demo
gazebo --verbose /opt/ros/humble/share/gazebo_ros/worlds/empty_world.sdf
```

### macOS Installation

```bash
# Using Homebrew
brew install gazebo

# Verify installation
gazebo --version
```

### Windows WSL2 Setup

```bash
# Inside WSL2 Ubuntu, follow Ubuntu installation above

# Important: Set display for GUI
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# Launch Gazebo
gazebo --verbose &
```

### Docker Container

```bash
# Use official Gazebo image
docker pull gazebo:harmonic

# Run with X11 forwarding
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/.gazebo:/home/gazebo/.gazebo \
  gazebo:harmonic

# Or combined with ROS 2
docker pull osrf/ros:humble-desktop
```

### Verify Complete Installation

```bash
# Test 1: Launch empty world
gazebo --verbose

# Test 2: Run Gazebo + ROS 2 demo
ros2 launch gazebo_ros gazebo.launch.py

# Test 3: Verify plugins load
gazebo --verbose -e ode /opt/ros/humble/share/gazebo_ros/worlds/empty_world.sdf
```

## Basic Concepts

Gazebo's architecture is built around three core abstractions: Worlds, Models, and Links. Understanding these hierarchies is essential for building realistic simulations.

### World
The World is the top-level container for everything in your simulation. It defines the physics engine, gravity, lighting, and serves as a container for all models and sensors.

**World Structure**:
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>        <!-- Standard Earth gravity -->
      <max_step_size>0.001</max_step_size> <!-- 1ms timesteps for accuracy -->
      <real_time_factor>1.0</real_time_factor> <!-- 1:1 with wall clock -->
      <real_time_update_rate>1000</real_time_update_rate> <!-- 1000 Hz -->
    </physics>

    <!-- Plugins that run in the world context -->
    <plugin filename="libgazebo_ros_init.so" name="ros_init_plugin"/>
    <plugin filename="libgazebo_ros_factory.so" name="factory"/>

    <!-- Lighting configuration -->
    <light name="sun">
      <pose>10 10 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground plane (required) -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your robots and objects -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 1 0 0 0</pose> <!-- x y z roll pitch yaw -->
    </include>
  </world>
</sdf>
```

**Physics Engines Available**:
- **ODE (Open Dynamics Engine)**: Default, good balance of speed/accuracy
- **Bullet**: Fast, less accurate, good for large-scale simulations
- **DART**: Most accurate, slower, best for control validation
- **Simbody**: Biomechanics-focused, specialized use cases

**Key Physics Parameters**:
- **gravity**: Usually [0, 0, -9.81] m/s² (Earth), adjust for other planets
- **max_step_size**: Smaller = more accurate but slower (0.001-0.01 typical)
- **real_time_factor**: 1.0 = simulation runs at real-time speed
- **real_time_update_rate**: Hz of physics updates (1000 Hz typical)

### Models
Models are reusable components: robots, objects, buildings, furniture. Each model contains links (rigid bodies) and joints (connections).

**Model Hierarchy**:
```
Model
  ├── Link (rigid body)
  │    ├── Inertial (mass properties)
  │    ├── Collision (physics)
  │    ├── Visual (graphics)
  │    └── Sensors
  ├── Link (another rigid body)
  └── Joint (connection between links)
```

**Complete Robot Model Example**:
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="my_robot">
    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>  <!-- Rotational inertia around x-axis -->
          <iyy>0.1</iyy>  <!-- Rotational inertia around y-axis -->
          <izz>0.2</izz>  <!-- Rotational inertia around z-axis -->
        </inertia>
      </inertial>

      <!-- Collision shape for physics -->
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size> <!-- Width, depth, height -->
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>   <!-- Friction coefficient -->
              <mu2>0.8</mu2> <!-- Alternative friction direction -->
            </ode>
          </friction>
        </surface>
      </collision>

      <!-- Visual appearance (for rendering) -->
      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1.0</ambient>
          <diffuse>0.8 0.8 0.8 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
        </material>
      </visual>
    </link>

    <!-- Wheel link -->
    <link name="wheel_left">
      <pose relative_to="base_link">-0.15 0.1 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <collision>
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>

      <visual>
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- Another wheel -->
    <link name="wheel_right">
      <pose relative_to="base_link">-0.15 -0.1 0 0 0 0</pose>
      <!-- Similar structure to wheel_left -->
    </link>

    <!-- Joint connecting wheels to base -->
    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left</child>
      <axis>
        <xyz>0 1 0</xyz> <!-- Rotate around y-axis -->
      </axis>
      <limit>
        <effort>10</effort>   <!-- Max torque (Nm) -->
        <velocity>10</velocity> <!-- Max speed (rad/s) -->
      </limit>
    </joint>

    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <limit>
        <effort>10</effort>
        <velocity>10</velocity>
      </limit>
    </joint>

    <!-- Plugin to control the wheels from ROS 2 -->
    <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive">
      <ros>
        <namespace>/robot</namespace>
        <argument>cmd_vel:=cmd_vel</argument>
        <argument>odom:=odom</argument>
      </ros>
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.2</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
    </plugin>
  </model>
</sdf>
```

### Links (Rigid Bodies)
A Link is a rigid body that doesn't deform. Each link has:

**Inertial Properties** (for physics):
```xml
<inertial>
  <mass>1.0</mass>        <!-- kg -->
  <pose>0 0 0.1 0 0 0</pose> <!-- Offset of center of mass -->
  <inertia>
    <ixx>0.1</ixx>  <!-- Moment of inertia Ixx -->
    <ixy>0.0</ixy>  <!-- Product of inertia Ixy -->
    <ixz>0.0</ixz>
    <iyy>0.1</iyy>
    <iyz>0.0</iyz>
    <izz>0.1</izz>
  </inertia>
</inertial>
```

**Collision Shapes** (physics simulation):
- **Box**: Rectangular, good for robots and objects
- **Sphere**: Ball-shaped, efficient for physics
- **Cylinder**: Wheels, poles, pipes
- **Capsule**: Rounded cylinder, smooth collisions
- **Mesh**: Complex shapes from 3D models (slower)
- **Plane**: Infinite flat surface (ground)

**Visual Meshes** (for rendering):
- Can be different from collision shape (simpler collision = faster)
- COLLADA (.dae), Wavefront (.obj), STL (.stl) formats
- Materials: color, texture, reflectance

### Joints
Joints connect links together. Each joint can have limits, friction, and dynamics.

**Joint Types**:
- **revolute**: Rotates around an axis (1 DOF)
- **prismatic**: Slides along an axis (1 DOF)
- **ball**: Rotates in all directions (3 DOF)
- **universal**: Two perpendicular rotations (2 DOF)
- **screw**: Rotation + translation combination
- **fixed**: Rigid connection (0 DOF)

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

## Best Practices for Effective Simulation

### Model Organization and Structure

**Standard Library Usage**:
Gazebo includes hundreds of pre-built models. Start with these to save time:

```bash
# List available models
find /usr/share/gazebo-*/models -type d -maxdepth 1

# Use models in your world
<include>
  <uri>model://ground_plane</uri>
</include>
<include>
  <uri>model://sun</uri>
</include>
<include>
  <uri>model://cube_20k</uri>
  <pose>1 0 0.5 0 0 0</pose>
</include>
```

**Custom Model Organization**:
```bash
my_robots_pkg/
├── models/
│   ├── my_robot/
│   │   ├── model.sdf       # Model definition
│   │   ├── model.config    # Metadata
│   │   └── meshes/         # 3D models
│   └── my_gripper/
├── worlds/
│   ├── empty.world.sdf     # Minimal world
│   ├── warehouse.world.sdf # Complex scene
│   └── pick_and_place.world.sdf
└── launch/
    └── simulation.launch.py
```

**Model Configuration File** (`model.config`):
```xml
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  <description>
    A differential drive robot with camera and gripper.
  </description>
</model>
```

### Physics Tuning for Accuracy vs Speed

**Common Physics Issues and Solutions**:

**Issue 1: Simulation Too Slow**
```xml
<!-- Reduce physics accuracy for speed -->
<physics type="ode">
  <!-- Increase timestep (less accurate but faster) -->
  <max_step_size>0.01</max_step_size>
  <!-- Reduce iterations per step -->
  <ode>
    <solver>
      <type>quick</type>  <!-- Faster than default -->
      <iters>20</iters>   <!-- Fewer iterations -->
    </solver>
  </ode>
</physics>
```

**Issue 2: Unstable Physics (jittering, flying objects)**
```xml
<!-- Improve stability -->
<physics type="ode">
  <!-- Smaller timesteps for better stability -->
  <max_step_size>0.001</max_step_size>
  <!-- More iterations for constraint solving -->
  <ode>
    <solver>
      <type>world</type>
      <iters>50</iters>
    </solver>
    <constraints>
      <cfm>0</cfm>         <!-- Contact force mixing -->
      <erp>0.1</erp>       <!-- Error reduction parameter -->
      <contact_max_correcting_vel>10</contact_max_correcting_vel>
    </constraints>
  </ode>
</physics>
```

**Issue 3: Unrealistic Joint Behavior**
```xml
<!-- Damping and friction help stability -->
<joint>
  <damping>0.01</damping>     <!-- Velocity damping -->
  <friction>0.1</friction>    <!-- Joint friction -->
  <limit>
    <effort>100</effort>      <!-- Max torque -->
    <velocity>10</velocity>   <!-- Max speed -->
    <lower>0</lower>          <!-- Rotation limits -->
    <upper>1.57</upper>       <!-- π/2 radians -->
  </limit>
</joint>
```

### Debugging Simulation

**Visual Debugging in Gazebo GUI**:
1. **Wireframe Mode**: View only collisions without visuals
   - View → Wireframe in Gazebo GUI
2. **Center of Mass**: Visualize CoM of each link
   - View → Show Center of Mass
3. **Inertia**: Display inertia tensors
   - View → Show Inertia
4. **Normals**: Surface normals for collision shapes
   - View → Show Normals

**Programmatic Debugging with ROS 2**:
```python
# Subscribe to joint states to debug kinematics
def joint_callback(msg):
    print(f"Joint positions: {msg.position}")
    print(f"Joint velocities: {msg.velocity}")
    print(f"Joint efforts: {msg.effort}")

node.create_subscription(JointState, '/robot/joint_states', joint_callback, 10)

# Monitor sensor output
def camera_callback(msg):
    print(f"Camera: {msg.width}x{msg.height}, {msg.encoding}")

node.create_subscription(Image, '/robot/camera/image_raw', camera_callback, 10)
```

**Logging Physics Data**:
```bash
# Record all topics to a rosbag
ros2 bag record -a -o simulation_log

# Analyze later
ros2 bag info simulation_log
ros2 bag play simulation_log

# Extract specific data
ros2 bag play simulation_log | grep "joint_states"
```

### Sim-to-Real Transfer

**Domain Randomization** (reduce sim-to-real gap):
```xml
<!-- Add randomization to model -->
<plugin filename="libgazebo_domain_randomizer.so" name="domain_randomizer">
  <!-- Randomize mass ±10% -->
  <mass mean="1.0" sigma="0.1"/>
  <!-- Randomize friction ±20% -->
  <friction mean="0.8" sigma="0.16"/>
  <!-- Randomize joint damping -->
  <damping mean="0.01" sigma="0.002"/>
  <!-- Randomize colors (visual only) -->
  <color_randomize>true</color_randomize>
</plugin>
```

**Comparison Workflow**:
1. Train controller in sim with domain randomization
2. Deploy to real hardware
3. Record performance metrics
4. Adjust simulation parameters to match real performance
5. Re-train if performance gap > 5%

### Advanced Optimization Techniques

**GPU Acceleration**:
```bash
# Use CUDA-accelerated physics
gazebo --verbose -e bullet \
  -g /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libgazebo_ros.so
```

**Parallel Simulation**:
```bash
# Run multiple simulations in parallel for training
for i in {1..8}; do
  gazebo --physics ode -u world_${i}.sdf &
done
```

**Headless Operation** (no graphics):
```bash
# Run without rendering for speed (100x+ faster)
LIBGL_ALWAYS_INDIRECT=1 gazebo --verbose -u -s libgazebo_ros_init.so
```

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
