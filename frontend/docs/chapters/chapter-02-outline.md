# Chapter 2: Robot Simulation & AI Perception

## Metadata

**Chapter Number**: 2
**Full Title**: Robot Simulation & AI Perception: From Physics Engines to Vision-Based Navigation
**Estimated Length**: 3000-3500 words (8-10 pages)
**Estimated Reading Time**: 45-60 minutes
**Course Coverage**: Weeks 6-10 (Gazebo & Unity simulation, NVIDIA Isaac Platform, AI perception fundamentals)
**Prerequisites**: Chapter 1 (ROS 2 fundamentals, URDF definitions, sensor basics)
**Target Audience**: Undergraduate robotics students, practitioners learning simulation and perception systems

---

## Learning Outcomes

Upon completing this chapter, students will be able to:

1. **Design and execute physics simulations** of robotic systems using Gazebo and NVIDIA Isaac Sim, including proper sensor simulation and environment configuration.

2. **Implement visual perception pipelines** including sensor simulation (RGB-D cameras, LiDAR), point cloud processing, and SLAM (Simultaneous Localization and Mapping) algorithms.

3. **Deploy navigation stacks** using Nav2 in simulated environments, including path planning, costmap generation, and behavior tree-based autonomous navigation.

4. **Analyze and optimize sim-to-real transfer** by understanding the physics-perception gap between simulation and real robots, and applying domain adaptation techniques.

5. **Integrate NVIDIA Isaac Sim workflows** for synthetic data generation, digital twins, and reinforcement learning training pipelines.

---

## Section Hierarchy

### 2.1 Introduction: Why Simulation Matters for Physical AI

#### 2.1.1 The Simulation-Reality Pipeline
- Definition of simulation in robotics
- Role in design iteration, testing, and training
- Cost-benefit analysis: simulation vs. physical experiments
- Real-world examples: Tesla's Optimus training, Boston Dynamics testing

#### 2.1.2 From Open-Loop to Closed-Loop: Sensor Simulation
- Challenges in sensor simulation
- Why perfect sensors don't exist
- Bridging simulation-reality gap
- Importance of realistic sensor models

#### 2.1.3 AI Perception in Robotics
- Perception as bottleneck for autonomy
- Vision, LiDAR, IMU, tactile sensing
- SLAM as foundation for navigation
- Overview of perception pipeline

#### 2.1.4 Chapter Roadmap
- What students will build: Simulated robot in Gazebo, perception pipeline in ROS 2
- Technologies covered: Gazebo, Isaac Sim, VSLAM, Nav2
- Practical outcomes: End-to-end autonomous navigation

---

### 2.2 Physics Simulation Foundations: Gazebo

#### 2.2.1 Gazebo Architecture and Physics Engines

**Key Concepts**:
- Gazebo ecosystem: server, client, plugins
- Physics engines: ODE, Bullet, Dart, SIMBODY comparison
- Simulation loop: world update, sensor measurement, actuation
- Time stepping and numerical stability

**Equations/Formulations**:
- Basic physics update: $\mathbf{F} = m\mathbf{a}$
- Rigid body dynamics: $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau}$
- Constraint forces (contacts, joints)

#### 2.2.2 World Configuration and SDF (Simulation Description Format)

**Key Concepts**:
- SDF vs. URDF comparison
- World files structure
- Physics parameters tuning (gravity, time step, solver iteration)
- Material properties (friction, restitution, surface characteristics)

**Practical Elements**:
- Setting up ground planes and obstacles
- Configuring lighting and camera viewpoint
- Weather and environmental effects

#### 2.2.3 Sensor Simulation in Gazebo

**Sensors to Cover**:
- **RGB Camera**: Rendering pipeline, lens distortion, noise models
- **Depth Camera (RGB-D)**: Z-buffer depth rendering, noise injection
- **LiDAR (Laser Range Finder)**: Ray casting, range noise, beam divergence
- **IMU (Inertial Measurement Unit)**: Gyroscope/accelerometer drift, bias, white noise
- **Contact/Force Sensors**: Normal force, contact point detection

**Key Equations**:
- Camera projection: $\mathbf{p} = \mathbf{K}[\mathbf{R} | \mathbf{t}]\mathbf{P}$ (intrinsic/extrinsic calibration)
- Depth noise model: $z_{noisy} = z_{true} + \mathcal{N}(0, \sigma^2)$
- IMU measurement: $\mathbf{a}_{meas} = \mathbf{R}^T(\mathbf{a}_{true} + \mathbf{g}) + \mathbf{b}_a + \mathbf{w}_a$

#### 2.2.4 ROS 2 Integration with Gazebo

**Key Concepts**:
- ros2_gazebo bridge and plugins
- Publishing sensor data to ROS 2 topics
- Subscribing to actuator commands (velocity, torque)
- TF (Transform) broadcasting for kinematic chains
- Timing synchronization between ROS 2 and Gazebo

**Example Plugins**:
- `gazebo_ros2_control` for joint/motor control
- Custom plugins for sensor publishing
- Physics plugins for friction, contact modeling

---

### 2.3 Advanced Simulation: NVIDIA Isaac Sim and Isaac Platform

#### 2.3.1 Introduction to NVIDIA Isaac Ecosystem

**Key Concepts**:
- Isaac Platform vs. Isaac Sim vs. Isaac Workspace
- Omniverse foundation and USD (Universal Scene Description)
- GPU-accelerated physics and rendering
- Comparison with Gazebo (fidelity, performance, scalability)

**Advantages**:
- Photorealistic rendering for vision model training
- Large-scale parallel simulation (thousands of robots)
- Native synthetic data generation
- Isaac ROS bridges for ROS 2 integration

#### 2.3.2 Isaac Sim Fundamentals

**Stage and Asset Management**:
- USD stage structure and prim hierarchy
- Asset libraries and catalog organization
- Physics schemas and rigid body definition
- Joint configuration and constraints

**Physics in Isaac Sim**:
- PhysX physics engine (GPU-accelerated)
- Articulation schemas (multi-body systems)
- Joint types and drive properties
- Contact and friction modeling (improved over CPU)

**Key Formulations**:
- Implicit Euler integration: $\mathbf{v}_{n+1} = \mathbf{v}_n + dt \cdot \mathbf{M}^{-1}(\mathbf{f}_{ext} + \mathbf{f}_{constraint})$
- Constraint solving using Projected Gauss-Seidel (PGS)

#### 2.3.3 Sensor Simulation in Isaac Sim

**Advanced Features**:
- **RTX-accelerated rendering**: Ray-traced camera simulation
- **Synthetic data generation**: Randomization, domain randomization, multi-modal output
- **LiDAR simulation**: GPU-accelerated ray casting with realistic beam patterns
- **Physics-based depth**: Real-time rendering to depth texture

**Key Capabilities**:
- Semantic segmentation maps (per-pixel class labels)
- Instance segmentation (per-object ID)
- Bounding box generation
- HDR rendering for robust vision models
- Distortion, noise, and weather effects (rain, fog, snow)

#### 2.3.4 Synthetic Data Generation Pipeline

**Use Cases**:
- Training object detection models without manual labeling
- Sim-to-real transfer with domain randomization
- Testing vision algorithms at scale (10k+ scenarios)

**Domain Randomization Factors**:
- Lighting conditions (intensity, color, direction)
- Camera intrinsics (focal length, noise)
- Material properties (colors, roughness, reflectance)
- Pose variations
- Background and clutter

**Example Workflow**:
1. Define robot task (bin picking, navigation)
2. Configure scene with randomization parameters
3. Render and annotate synthetically
4. Train vision model on synthetic dataset
5. Deploy and fine-tune on real robot

#### 2.3.5 Isaac ROS Integration

**Key Components**:
- Isaac ROS2 extensions for Omniverse
- Bridge nodes for sensor data (camera, LiDAR, odometry)
- Simulation clock synchronization
- Bridge to external ROS 2 stacks

**Workflow**:
- Launch Isaac Sim with ROS 2 bridge
- Subscribe to ROS 2 topics in simulation
- Publish sensor data to standard ROS 2 message types
- Integrate with Nav2, SLAM, or other middleware

---

### 2.4 Visual Perception Foundations

#### 2.4.1 Camera Models and Calibration

**Pinhole Camera Model**:
- Intrinsic parameters: focal length, principal point, skew
- Extrinsic parameters: position and orientation relative to world
- Projection equation: $\begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = \frac{1}{Z} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}$

**Distortion Models**:
- Radial distortion (barrel/pincushion)
- Tangential distortion
- Distortion coefficients and correction

**Depth Cameras (RGB-D)**:
- Structured light, Time-of-Flight, stereo vision
- Depth accuracy and range limits
- Noise characteristics in simulator

#### 2.4.2 Point Clouds and 3D Perception

**Fundamentals**:
- Point cloud representation: (x, y, z) + intensity/color
- ROS 2 message format: `sensor_msgs/PointCloud2`
- Common operations: filtering, downsampling, segmentation

**Point Cloud Processing**:
- Statistical outlier removal
- Voxel downsampling (spatial sampling)
- Plane segmentation (RANSAC)
- Cluster extraction (Euclidean clustering)

**Key Libraries**:
- PCL (Point Cloud Library)
- Open3D
- ROS 2 point_cloud_to_laserscan (for LiDAR to 2D conversion)

#### 2.4.3 LiDAR and 2D/3D Range Sensing

**LiDAR Fundamentals**:
- Principle: time-of-flight laser scanning
- Beam pattern and angular resolution
- Range accuracy and dropout zones
- Sparse vs. dense point clouds

**Common LiDAR Models in Robotics**:
- Livox Mid-360 (16-channel, ~10m range)
- Sick OS1 (64-channel, ~120m range)
- Velodyne VLP-32 (32-channel, ~100m range)

**ROS 2 Integration**:
- Publishing to `/scan` (2D LaserScan) or `/points` (3D PointCloud2)
- Frame ID and timestamp management
- Time synchronization critical for SLAM

#### 2.4.4 Feature Detection and Matching

**Feature-Based Methods**:
- **SIFT** (Scale-Invariant Feature Transform): rotation/scale invariant
- **ORB** (Oriented FAST and Rotated BRIEF): real-time, efficient
- **AKAZE**: features for constrained environments

**Descriptors and Matching**:
- Descriptor matching: Hamming distance, L2 distance
- Outlier rejection: Lowe's ratio test, RANSAC

**Deep Learning Alternatives**:
- SuperPoint, LIFT, R2D2: learned feature detectors/descriptors
- More robust but computationally expensive

---

### 2.5 Simultaneous Localization and Mapping (SLAM)

#### 2.5.1 SLAM Problem Formulation

**Core Challenge**:
- Robot starts with unknown pose in unknown environment
- Must use onboard sensors to build map while localizing within it
- Chicken-and-egg problem: need map for localization, need localization to build map

**SLAM Types**:
- **Visual SLAM (V-SLAM)**: Uses camera images
- **LiDAR SLAM**: Uses laser range measurements
- **Multi-sensor SLAM**: Fuses cameras, LiDAR, IMU

**State Representation**:
- Robot pose: $\mathbf{x}_r = [x, y, \theta]^T$ (2D) or $[\mathbf{p}, \mathbf{R}]$ (3D)
- Map landmarks: $\{\mathbf{m}_1, \mathbf{m}_2, ..., \mathbf{m}_n\}$
- State vector: $\mathbf{x} = [x_r, \mathbf{m}]^T$

#### 2.5.2 Visual SLAM (V-SLAM) Algorithms

**MonoSLAM / StereoSLAM**:
- Monocular SLAM: single camera (scale ambiguity issue)
- Stereo SLAM: two cameras (known baseline)
- Key steps: feature detection, feature matching, pose estimation

**ORB-SLAM Family**:
- **ORB-SLAM2**: monocular, stereo, RGB-D SLAM
- Tracking: feature matching + pose estimation via PnP
- Local mapping: bundle adjustment, loop closure detection
- Optimization: BA (Bundle Adjustment) refinement

**Key Equations**:
- Epipolar constraint: $\mathbf{x}_2^T \mathbf{F} \mathbf{x}_1 = 0$ (stereo matching)
- PnP (Perspective-n-Point): $\min_{\mathbf{R},\mathbf{t}} \sum_{i=1}^{n} ||\mathbf{x}_i - \mathbf{K}[\mathbf{R}|\mathbf{t}]\mathbf{X}_i||^2$
- Bundle Adjustment: $\min \sum_{i,j} ||\mathbf{x}_{ij} - \pi(\mathbf{X}_i, \mathbf{P}_j)||^2 + \text{regularization}$

#### 2.5.3 LiDAR SLAM Algorithms

**Scan-to-Scan Matching**:
- Point-to-point ICP (Iterative Closest Point)
- Point-to-plane ICP (more robust to rotation errors)
- Normal estimation and correspondence search

**Scan-to-Map Matching**:
- Incremental map building
- Grid maps vs. feature-based maps
- Occupancy grid representation

**Popular LiDAR SLAM Systems**:
- **LOAM** (Lidar Odometry and Mapping)
- **CARTOGRAPHER** (Google's real-time SLAM)
- **FAST-LIO2**: Real-time LiDAR-Inertial Odometry

#### 2.5.4 Loop Closure and Global Optimization

**Loop Closure Detection**:
- Problem: when robot revisits known location, must correct drift
- Methods:
  - Place recognition: bag-of-words matching
  - Visual place recognition: NetVLAD, DenseVLAD
  - LiDAR loop closure: scan context matching

**Global Optimization**:
- Pose Graph Optimization
- Constraint adjustment when loop is detected
- Solvers: g2o, iSAM2, Ceres Solver

**Mathematical Formulation**:
- Pose graph: vertices = poses, edges = relative constraints
- Objective: $\min_{\mathbf{x}} \sum_{(i,j) \in \mathcal{E}} ||\mathbf{T}_{ij}^{-1} \mathbf{T}_i^{-1} \mathbf{T}_j||^2_{\Sigma}$

#### 2.5.5 Multi-Sensor Fusion (Visual-Inertial SLAM)

**Sensor Fusion Principles**:
- Complementary sensor properties: camera (drift-free, prone to fast motion blur), IMU (high frequency, drifts)
- Extended Kalman Filter (EKF) formulation
- Tightly-coupled vs. loosely-coupled fusion

**Visual-Inertial Odometry (VIO)**:
- **VINS-Mono**: monocular visual-inertial system
- **Kimera**: real-time 3D metric-semantic mapping
- IMU preintegration for efficiency

**Key Equations**:
- IMU kinematic model: $\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}\mathbf{a}_k \Delta t^2$
- IMU measurement: $\mathbf{a}_{meas} = \mathbf{R}^T(\mathbf{a}_{true} + \mathbf{g}) + \mathbf{b}_a + \mathbf{w}_a$
- EKF prediction: $\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)$

---

### 2.6 Autonomous Navigation with Nav2

#### 2.6.1 Navigation Stack Architecture

**Nav2 Overview**:
- Modern successor to ROS 1 Navigation Stack
- Modular, composable architecture
- Plugins for different algorithms and hardware

**Core Components**:
- **Map Server**: maintains occupancy grid or costmap
- **Costmap Layers**: static map, obstacles, inflation
- **Global Planner**: long-horizon path planning (NavFn, Theta*, SmacPlanner)
- **Local Planner**: collision avoidance and trajectory tracking (DWB, RotStabilizer, MPPI)
- **Behavior Server**: Behavior trees for navigation logic
- **Recovery Behaviors**: navigate on stuck, spin recovery

#### 2.6.2 Costmap Representation and Layers

**Occupancy Grid**:
- 2D grid cells: free (0), unknown (-1), occupied (255)
- Resolution: typically 5cm per cell
- Cost scale: 0 (free) to 254 (occupied), 255 (lethal)

**Costmap Layers**:
- **Static Layer**: from map file, fixed obstacles
- **Obstacle Layer**: detected obstacles from sensors (LiDAR, camera)
- **Inflation Layer**: cost increases near obstacles (collision radius)
- **Voxel Layer**: 3D representation for tall obstacles

**Key Concepts**:
- Cell inflation radius: $r_{inflation} = r_{robot} + safety\_margin$
- Cost function: $cost(d) = \max(0, (r_{inflation} - d) \cdot \text{scale})$

#### 2.6.3 Path Planning Algorithms

**Global Planners** (off-line, high-level):
- **NavFn** (Navigation Function): gradient-descent based
- **Theta\*** (Any-angle): smooths diagonal movements
- **SmacPlanner** (SMAC): Sampling-based Motion Primitive A*

**Algorithm: A* Path Planning**:
$$f(n) = g(n) + h(n)$$
where $g(n)$ = cost from start, $h(n)$ = heuristic to goal

**Local Planners** (on-line, collision avoidance):
- **DWB** (Dynamic Window Approach): velocity-space sampling
- **RotStabilizer**: angular acceleration limiting
- **MPPI** (Model Predictive Path Integral): sampling-based MPC

#### 2.6.4 Behavior Trees for Navigation

**Why Behavior Trees**:
- Reactive and hierarchical execution
- Flexible recovery strategies
- Composable navigation behaviors

**Tree Structure**:
- **Selectors** (OR): try children until success
- **Sequences** (AND): execute children in order
- **Actions**: primitive behaviors (move, rotate, dock)
- **Conditions**: checks (goal reached, path valid, etc.)

**Example Tree (Pseudo-BT)**:
```
Root
├── Sequence (Main Nav)
│   ├── ComputePath
│   ├── FollowPath
│   └── Reached Goal?
└── Selector (Error Handling)
    ├── Spin Recovery
    ├── Back-up Recovery
    └── Pause
```

#### 2.6.5 Integration with SLAM and Sensor Data

**Data Flow**:
- SLAM publishes: `/tf` (odometry), `/map` (occupancy grid), `/amcl_pose` (global localization)
- Obstacle detectors publish: `/pointcloud`, `/scan`
- Nav2 consumes: costmaps, odometry, goal, sensor data
- Nav2 publishes: `/cmd_vel`, navigation status

**Odometry Sources**:
- Visual odometry from VSLAM
- Wheel odometry (encoder-based)
- IMU-based odometry
- Fusion via extended Kalman filter

---

### 2.7 Putting It Together: End-to-End Autonomous Navigation System

#### 2.7.1 System Integration Workflow

**Architecture Diagram Description**:
*[Figure 2.1: End-to-end autonomous navigation architecture showing: (1) Gazebo/Isaac Sim simulation with physics and sensor simulation, (2) Sensor drivers publishing to `/camera/image`, `/lidar/scan`, `/imu/data` topics, (3) VSLAM node consuming images and publishing `/tf`, `/map`, (4) Nav2 stack consuming `/map` and sensor data, publishing `/cmd_vel`, (5) Robot controller consuming `/cmd_vel` and publishing odometry]*

**Component Integration**:
1. Simulation produces realistic sensor data
2. SLAM processes sensors → estimate pose + map
3. Navigation stack consumes map → plans path → outputs velocity commands
4. Controller executes commands → affects robot in simulation
5. Feedback loop closes through odometry

#### 2.7.2 Sim-to-Real Challenges and Mitigation

**Physics Gap**:
- Simulated friction ≠ real friction
- Contact dynamics simplified
- Actuator limitations (backlash, delay) often ignored

**Perception Gap**:
- Simulated camera noise ≠ real noise
- Motion blur, rolling shutter not modeled
- LiDAR shadows, specular reflections underestimated

**Mitigation Strategies**:
- **Domain Randomization**: randomize all parameters during training
- **System Identification**: learn mapping between simulated and real parameters
- **Transfer Learning**: pre-train in simulation, fine-tune on real robot
- **Robust Control**: design controller tolerant to model mismatch

#### 2.7.3 Performance Metrics

**Navigation Success Rate**:
$$\text{Success Rate} = \frac{\text{Successful goal reaches}}{\text{Total trials}}$$

**Path Quality**:
- Path length relative to optimal
- Smoothness (curvature metrics)
- Collision clearance to obstacles

**Computational Efficiency**:
- CPU/GPU utilization
- End-to-end latency: sensor capture → command execution
- Real-time factor (wall-time per sim-time)

---

### 2.8 Summary and Key Takeaways

#### Major Concepts Covered

1. **Physics Simulation**:
   - Gazebo as standard ROS 2 simulation platform
   - Physics engines and numerical stability
   - Sensor simulation importance

2. **Advanced Simulation**:
   - NVIDIA Isaac Sim for photorealistic rendering
   - Synthetic data generation pipelines
   - GPU-accelerated physics and perception

3. **Visual Perception**:
   - Camera models, calibration, depth sensing
   - Point cloud processing fundamentals
   - LiDAR principles and applications

4. **SLAM Algorithms**:
   - Visual SLAM (ORB-SLAM family)
   - LiDAR SLAM (LOAM, Cartographer)
   - Loop closure and multi-sensor fusion

5. **Autonomous Navigation**:
   - Nav2 architecture and components
   - Costmaps, path planning, local control
   - Behavior trees for reactive navigation

6. **System Integration**:
   - End-to-end autonomous system design
   - Sim-to-real transfer strategies
   - Performance evaluation metrics

#### Critical Insights

- **Simulation is essential but imperfect**: Fidelity matters, but perfect realism is unnecessary; focus on the physics/perception phenomena relevant to your task.

- **SLAM is foundational**: Navigation is impossible without accurate localization; invest in robust SLAM algorithms.

- **Sensor selection drives architecture**: LiDAR enables robust mapping; vision enables place recognition and object detection—choose based on environment.

- **Sim-to-real requires discipline**: Domain randomization and robust control are non-negotiable for real-world deployment.

#### Common Pitfalls to Avoid

- Simulating with unrealistic sensor parameters (e.g., no noise)
- Ignoring contact dynamics and friction
- Over-tuning controllers to specific simulation settings
- Under-estimating perception latency effects on control
- Deploying untested SLAM systems in novel environments

---

## Key Concepts and Terminology

### Equations and Mathematical Formulations

| Concept | Equation | Context |
|---------|----------|---------|
| **Robot Dynamics** | $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau}$ | Physics simulation in Gazebo |
| **Camera Projection** | $\mathbf{p} = \mathbf{K}[\mathbf{R} \| \mathbf{t}]\mathbf{P}$ | Perspective camera model |
| **IMU Measurement Model** | $\mathbf{a}_{meas} = \mathbf{R}^T(\mathbf{a}_{true} + \mathbf{g}) + \mathbf{b}_a + \mathbf{w}_a$ | Sensor noise modeling |
| **PnP (Perspective-n-Point)** | $\min_{\mathbf{R},\mathbf{t}} \sum \|\|\mathbf{x}_i - \mathbf{K}[\mathbf{R}\|\mathbf{t}]\mathbf{X}_i\|\|^2$ | SLAM pose estimation |
| **A* Cost Function** | $f(n) = g(n) + h(n)$ | Global path planning |
| **Costmap Inflation** | $cost(d) = \max(0, (r_{inflation} - d) \cdot scale)$ | Navigation costmap layers |
| **Pose Graph Optimization** | $\min_{\mathbf{x}} \sum_{(i,j)} \|\|\mathbf{T}_{ij}^{-1} \mathbf{T}_i^{-1} \mathbf{T}_j\|\|_{\Sigma}^2$ | SLAM loop closure |

### Core Algorithms

| Algorithm | Domain | Key Steps | Complexity |
|-----------|--------|-----------|-----------|
| **Gazebo Physics Loop** | Simulation | 1) Update forces, 2) Integrate dynamics, 3) Resolve collisions, 4) Publish state | O(n) bodies |
| **ORB-SLAM2** | V-SLAM | 1) Feature detection, 2) Feature matching, 3) Pose estimation, 4) BA, 5) Loop detection | O(n) keyframes |
| **ICP (Iterative Closest Point)** | LiDAR SLAM | 1) Find nearest neighbors, 2) Estimate transform, 3) Apply transform, 4) Iterate | O(n log n) |
| **A* Search** | Path Planning | 1) Initialize open set, 2) Pop lowest f(n), 3) Expand neighbors, 4) Update costs, repeat | O(n log n) |
| **Dynamic Window Approach** | Local Planning | 1) Sample velocities, 2) Simulate trajectory, 3) Score (progress + obstacles), 4) Select best | O(v * t) |
| **Behavior Tree Execution** | Navigation Control | 1) Traverse tree from root, 2) Execute actions, 3) Return status, 4) Check transitions | O(d) depth |

### Technical Terminology (for Glossary)

**Simulation & Physics**:
- Physics engine, rigid body, constraint, time stepping, numerical stability, ODE, collision detection
- Gazebo, SDF (Simulation Description Format), World file, Plugin

**Perception & Sensors**:
- RGB-D camera, point cloud, LiDAR, laser range finder, depth sensor, structured light, time-of-flight
- Feature detection, descriptor, feature matching, keypoint, scale-invariant

**SLAM**:
- Simultaneous Localization and Mapping (SLAM), visual SLAM, LiDAR SLAM, odometry
- Loop closure, place recognition, bundle adjustment, pose graph, keyframe
- Monocular, stereo, RGB-D SLAM
- Feature-based SLAM, direct SLAM (photometric error)

**Navigation**:
- Occupancy grid, costmap, inflation layer, global planner, local planner
- Path planning, A*, Dijkstra, RRT (Rapidly-exploring Random Tree)
- Behavior tree, selector, sequence, collision avoidance, dynamic window

**Integration & Concepts**:
- Sim-to-real, domain randomization, digital twin, sensor simulation, synthetic data
- Odometry, localization, mapping, navigation, autonomous system

---

## Worked Examples

### Worked Example 1: Setting Up a Simulated Robot in Gazebo

**Scenario**: You have a 2-DOF mobile robot (differential-drive base with rotating sensor head) and want to simulate it in Gazebo with camera and LiDAR sensors.

**Task Steps**:

1. **Create URDF with Sensors**
   - Define base_link (rectangular box for chassis)
   - Add wheels (cylinders on axle)
   - Add sensor_head (rotating link)
   - Add camera (child of sensor_head)
   - Add LiDAR (child of sensor_head)

2. **Configure Gazebo Plugins**
   - `gazebo_ros2_control` plugin: maps `/cmd_vel` → wheel torques
   - Camera plugin: publishes `/camera/image_raw` + `/camera/camera_info`
   - LiDAR plugin: publishes `/scan` (2D) or `/points` (3D point cloud)

3. **Create World File**
   - Load robot model at origin
   - Add ground plane and some obstacles
   - Set physics parameters (time step=0.001s, gravity=-9.81m/s²)
   - Configure Gazebo client (initial camera view)

4. **Launch and Verify**
   - `ros2 launch gazebo_ros gazebo.launch.py world:=my_world.sdf`
   - Verify sensors publishing: `ros2 topic list | grep -E 'camera|scan|points'`
   - View point clouds: `rviz2`

**Expected Outcome**: Robot simulated in Gazebo, sensors publishing data, ready for perception algorithm testing.

**Key Learning**: Proper URDF structure and Gazebo plugins are essential for simulation accuracy.

---

### Worked Example 2: Implementing a Simple V-SLAM System

**Scenario**: You want to implement monocular visual SLAM using ORB features to localize a mobile robot in a textured environment.

**Algorithm Outline**:
1. Capture sequential camera frames
2. Detect ORB features in each frame
3. Match features between consecutive frames
4. Estimate pose using PnP solver
5. Store keyframes and triangulate landmarks
6. Perform bundle adjustment periodically

**Python/ROS 2 Implementation Sketch**:

```python
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class SimpleVSLAM(Node):
    def __init__(self):
        super().__init__('simple_vslam')

        # ORB detector
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Camera intrinsics (from calibration)
        self.K = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)

        # State
        self.previous_frame = None
        self.previous_kpts = None
        self.previous_desc = None
        self.pose = np.eye(4)  # [R|t; 0|1]
        self.landmarks = []

        # ROS 2 setup
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Step 1: Detect features
        kpts, desc = self.orb.detectAndCompute(frame, None)

        if self.previous_desc is None:
            self.previous_frame = frame
            self.previous_kpts = kpts
            self.previous_desc = desc
            return

        # Step 2: Match features
        matches = self.bf_matcher.knnMatch(self.previous_desc, desc, k=2)

        # Apply Lowe's ratio test for outlier rejection
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < 10:
            self.get_logger().warning('Not enough feature matches')
            return

        # Step 3: Extract matched points
        pts_prev = np.float32([
            self.previous_kpts[m.queryIdx].pt for m in good_matches
        ]).reshape(-1, 1, 2)
        pts_curr = np.float32([
            kpts[m.trainIdx].pt for m in good_matches
        ]).reshape(-1, 1, 2)

        # Step 4: Estimate essential matrix
        E, mask = cv2.findEssentialMat(pts_prev, pts_curr, self.K, method=cv2.RANSAC)

        if E is None:
            self.get_logger().warning('Could not estimate essential matrix')
            return

        # Step 5: Recover pose
        _, R, t, mask = cv2.recoverPose(E, pts_prev, pts_curr, self.K, mask=mask)

        # Update pose (simple accumulation, no BA)
        T_new = np.eye(4)
        T_new[:3, :3] = R
        T_new[:3, 3] = t.flatten()
        self.pose = self.pose @ T_new

        # Publish odometry
        odom_msg = Odometry()
        # ... fill odom_msg from self.pose ...
        self.odom_pub.publish(odom_msg)

        # Update for next iteration
        self.previous_frame = frame
        self.previous_kpts = kpts
        self.previous_desc = desc
```

**Key Learning Points**:
- Feature matching is critical; outlier rejection via RANSAC essential
- Bundle adjustment required for drift-free long-term SLAM
- Real implementation (ORB-SLAM2) much more sophisticated

---

### Worked Example 3: Autonomous Navigation with Nav2

**Scenario**: You have a mobile robot in a simulated indoor environment (Gazebo with map) and want to navigate it autonomously to a goal location using Nav2.

**Setup Steps**:

1. **Prepare Environment**
   - Create occupancy grid map (from SLAM or manually in GIMP)
   - Save as `.pgm` image + `.yaml` metadata file
   - Verify in RViz2

2. **Launch Nav2 Stack**
   ```bash
   ros2 launch nav2_bringup navigation_launch.py \
       map:=./my_map.yaml \
       use_sim_time:=True
   ```

3. **Set Initial Pose**
   - Use RViz2 "2D Pose Estimate" button
   - Click on map at robot's actual location
   - Set orientation by dragging

4. **Send Goal**
   - Click "Nav2 Goal" button in RViz2
   - Click on target location
   - Observe path planning and execution

5. **Monitor Performance**
   ```bash
   ros2 topic echo /local_plan           # Local trajectory
   ros2 topic echo /planned_path          # Global path
   ros2 topic echo /navigation_result     # Success/failure
   ```

**Expected Behavior**:
- Global planner (NavFn) computes path around obstacles
- Local planner (DWB) follows path while avoiding dynamic obstacles
- Robot reaches goal with smooth, collision-free trajectory

**Tuning Parameters**:
- **Costmap inflation radius**: affects how close robot gets to walls
- **Local planner velocity/acceleration limits**: safety vs. agility tradeoff
- **Path smoothness parameter**: higher = smoother but longer paths

**Common Issues & Fixes**:
- Robot spins in place → costmap inflation too high, reduce it
- Path is jerky → increase path smoothness parameter
- Navigation fails → check map frame/TF tree with `ros2 run tf2_tools view_frames`

---

## Code Examples to Implement

### Example 1: Gazebo World and Robot Simulation

**File**: `gazebo/models/my_robot.urdf`
**Demonstrates**: URDF structure with sensors, Gazebo plugins for control and perception

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (chassis) -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="10" lower="-3.14" upper="3.14"/>
  </joint>
  <link name="left_wheel">
    <!-- ... wheel geometry ... -->
  </link>

  <!-- Camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.1"/>
  </joint>
  <link name="camera_link"/>

  <!-- Gazebo sensor plugin for camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>camera</namespace>
          <remapping>image_raw:=image_raw</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

**Learning Objective**: Understand URDF sensor attachment, Gazebo plugin configuration for ROS 2 integration.

---

### Example 2: ROS 2 SLAM Node (Simplified Visual Odometry)

**File**: `slam_node.py`
**Demonstrates**: Feature detection, optical flow, pose estimation, odometry publishing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Transform, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        # Feature detector (ORB)
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.bridge = CvBridge()
        self.prev_frame = None
        self.prev_kpts = None
        self.prev_desc = None
        self.pose = np.eye(4)
        self.K = None  # Camera intrinsics

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3).astype(np.float32)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect features
        kpts, desc = self.orb.detectAndCompute(gray, None)

        if self.prev_desc is None:
            self.prev_frame = gray
            self.prev_kpts = kpts
            self.prev_desc = desc
            return

        # Match features
        matches = self.matcher.match(self.prev_desc, desc)

        if len(matches) < 20:
            self.get_logger().warn('Too few matches, skipping frame')
            return

        # Extract matched points
        src_pts = np.float32([self.prev_kpts[m.queryIdx].pt for m in matches])
        dst_pts = np.float32([kpts[m.trainIdx].pt for m in matches])

        # Estimate homography and decompose
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if H is None:
            return

        # Decompose to R, t (simplified - assumes planar scene or distant landmarks)
        _, Rs, ts, ns = cv2.decomposeHomographyMat(H, self.K)

        # Use first valid decomposition
        T_new = np.eye(4)
        T_new[:3, :3] = Rs[0]
        T_new[:3, 3] = ts[0].flatten()

        self.pose = self.pose @ np.linalg.inv(T_new)  # Inverse for camera-to-world

        # Publish odometry
        self.publish_odometry(msg.header.stamp)

        # Update for next frame
        self.prev_frame = gray
        self.prev_kpts = kpts
        self.prev_desc = desc

    def publish_odometry(self, stamp):
        """Publish current pose as odometry and TF"""
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.pose[0, 3]
        odom_msg.pose.pose.position.y = self.pose[1, 3]
        odom_msg.pose.pose.position.z = self.pose[2, 3]

        # Orientation (from rotation matrix)
        from tf_transformations import quaternion_from_matrix
        q = quaternion_from_matrix(self.pose)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = 'base_link'
        t.transform = Transform()
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Learning Objective**: Implement visual odometry pipeline; understand feature-based motion estimation; ROS 2 sensor/odometry integration.

---

### Example 3: Nav2 Behavior Tree Configuration

**File**: `config/nav2_behavior.xml`
**Demonstrates**: Reactive navigation with error recovery via Behavior Trees

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="MainNav">
  <!-- Selector = OR logic, executes children until one succeeds -->
  <!-- Sequence = AND logic, executes all children in order -->

  <BehaviorTree ID="MainNav">
    <Selector name="RootSelector">
      <!-- Main navigation sequence -->
      <Sequence name="NavigateSequence">
        <Action ID="ComputePath"/>
        <Action ID="FollowPath"/>
        <Condition ID="GoalReached"/>
      </Sequence>

      <!-- Error recovery selector -->
      <Selector name="RecoverySelector">
        <Action ID="SpinRecovery"/>
        <Action ID="BackupRecovery"/>
        <Action ID="PauseAndRetry"/>
      </Selector>
    </Selector>
  </BehaviorTree>

  <!-- Node definitions -->
  <TreeNodesModel>
    <Action ID="ComputePath">
      <input_port name="goal">Goal position</input_port>
      <output_port name="path">Computed path</output_port>
    </Action>

    <Action ID="FollowPath">
      <input_port name="path">Path to follow</input_port>
    </Action>

    <Condition ID="GoalReached">
      <input_port name="goal">Target goal</input_port>
    </Condition>

    <Action ID="SpinRecovery">
      <!-- Spin in place to reorient -->
    </Action>

    <Action ID="BackupRecovery">
      <!-- Back up to unstuck -->
    </Action>
  </TreeNodesModel>
</root>
```

**Learning Objective**: Understand hierarchical task execution; reactive behavior design; recovery strategies for robust navigation.

---

## Practice Problems and Exercises

### Section 2.2 - Physics Simulation (Gazebo)

**P2.1** (Beginner): Given a URDF file of a 2-link planar arm, identify which link would have issues with large timesteps in Gazebo simulation and why. Suggest a fix.

**P2.2** (Beginner): Write the SDF snippet to add a box obstacle at position (1.0, 0.5, 0.5) with size 0.2 × 0.2 × 0.5 to a world file.

**P2.3** (Intermediate): Design a Gazebo sensor plugin configuration for an RGB-D camera with 640×480 resolution, 30 Hz frame rate, and realistic noise (standard deviation 0.01 m). What topics would it publish to?

**P2.4** (Intermediate): Explain why collision detection in Gazebo can fail if the timestep is too large. Provide a quantitative criterion based on robot velocity and link size.

**P2.5** (Advanced): Implement a Gazebo plugin that applies random friction perturbations at each simulation step to simulate wear or muddy terrain. How would you verify the plugin is working correctly?

---

### Section 2.3 - Isaac Sim and Synthetic Data

**P2.6** (Beginner): List three advantages of Isaac Sim over Gazebo for vision model training. Explain why photorealism matters.

**P2.7** (Beginner): Describe three domain randomization parameters you would vary when generating synthetic data for object detection in cluttered bins.

**P2.8** (Intermediate): Design a synthetic data generation pipeline for training a grasp detection model. Include: scene setup, randomization parameters, data annotation, training dataset size estimate.

**P2.9** (Advanced): Discuss the sim-to-real gap for a vision model trained on Isaac Sim synthetic data then deployed on a real robot. What failure modes are most likely, and how would you mitigate them?

---

### Section 2.4 - Perception Fundamentals

**P2.10** (Beginner): Given a camera with focal length f=500 pixels and a 3D point at (1m, 0, 5m), calculate its image coordinates assuming principal point at (320, 240).

**P2.11** (Intermediate): Explain the difference between monocular and stereo depth estimation. When would you choose each for a mobile robot?

**P2.12** (Intermediate): Implement a simple point cloud downsampling function (voxel grid) that reduces a cloud with 1M points to ~10k points. Describe the algorithm and estimate compute time.

**P2.13** (Advanced): Design a robust LiDAR-based plane segmentation algorithm for detecting floors in unstructured indoor environments. Account for slopes, carpets, and partial occlusion.

---

### Section 2.5 - SLAM

**P2.14** (Beginner): Explain why pure visual odometry (without loop closure) drifts over time and accumulates error. How does loop closure fix this?

**P2.15** (Intermediate): Compare ORB-SLAM2 (feature-based) and LSD-SLAM (direct) on the following criteria: robustness to motion blur, loop closure detection, map density, computational cost.

**P2.16** (Intermediate): Implement a simple 2D loop closure detector using bag-of-words (BoW) histogram matching on ORB descriptors. How would you avoid false loop closures?

**P2.17** (Advanced): Describe the visual-inertial odometry (VIO) formulation. How does IMU preintegration improve efficiency? Derive the measurement model for accelerometer and gyroscope.

---

### Section 2.6 - Nav2 Navigation

**P2.18** (Beginner): Draw a costmap with a 0.5m radius robot near a wall. Indicate: free space (0), obstacles (254), inflated region (128). Use 10cm grid cells, wall at x=1.0.

**P2.19** (Intermediate): Tune a Nav2 DWB local planner for a fast mobile base (1.5 m/s max velocity) in a cluttered environment. Which parameters would you adjust first? Explain tradeoffs.

**P2.20** (Advanced): Design a behavior tree for a mobile manipulator that must navigate to a shelf, grasp an object, and return to base while avoiding dynamic humans. Include error recovery strategies.

---

## Further Reading and References

### Foundational Textbooks

- **Robotics: Modelling, Planning and Control** (Siciliano, Sciavicco, Villani, Oriolo): Comprehensive coverage of kinematics and dynamics foundational to simulation.
- **Computer Vision: Algorithms and Applications** (Forsyth & Ponce): Visual perception and SLAM fundamentals.
- **Probabilistic Robotics** (Thrun, Burgard, Fox): Probabilistic approaches to localization, mapping, and navigation.

### Key Papers and Systems

**Simulation**:
- Gazebo official documentation: http://gazebosim.org/
- NVIDIA Isaac Sim: https://docs.omniverse.nvidia.com/isaacsim/latest/
- PhysX physics engine: https://developer.nvidia.com/physx

**Visual SLAM**:
- ORB-SLAM2 (Murillo et al., 2016): Feature-based SLAM for monocular, stereo, RGB-D
- LOAM (Zhang & Singh, 2014): LiDAR Odometry and Mapping
- Cartographer (Hess et al., 2016): Google's real-time SLAM system

**Navigation**:
- Nav2 official: https://navigation.ros.org/
- Dynamic Window Approach (Fox et al., 1997): Local obstacle avoidance
- Behavior Trees for Robotics: https://github.com/BehaviorTree/BehaviorTree.CPP

**Multi-Sensor Fusion**:
- VINS-Mono (Qin et al., 2018): Monocular visual-inertial system
- Kimera (Rosinol et al., 2021): Real-time 3D metric-semantic mapping

### ROS 2 Resources

- ROS 2 Official Documentation: https://docs.ros.org/
- ROS 2 Perception Meta-Package: sensor drivers, camera calibration
- Nav2 Tutorials: https://navigation.ros.org/setup_guides/index.html

### Online Courses and Tutorials

- Computer Vision & Robotics (Udacity): perception algorithms
- Modern C++ for Robotics: real-time implementation
- SLAM tutorials (YouTube): Visual SLAM, LiDAR SLAM walkthroughs

---

## Connection to Course Structure

**Prerequisite Chapter**:
- Chapter 1: Introduction to Physical AI & ROS 2 (robot definition, sensor basics, ROS 2 communication)

**Follow-up Chapters**:
- Chapter 3: Vision-Language-Action for Robotics (uses perception pipeline from this chapter for high-level planning and manipulation)

**Complementary Topics** (not in core textbook but referenced):
- Advanced control theory (needed for understanding controller tuning)
- 3D geometry and linear algebra refreshers
- Real-time systems concepts (for deployment considerations)

---

## Learning Progression Hints for Instructors

1. **Week 6-7 (Gazebo)**: Focus on simulation mechanics, not perfect realism. Students should build confidence with tool before worrying about physics fidelity.

2. **Week 8-9 (Isaac Sim)**: Emphasize synthetic data generation benefits; show concrete examples (e.g., trained model on Isaac data works on real robot).

3. **Week 10 (SLAM)**: Heavy theory week; use visual demos (rviz2 loop closures) to maintain engagement. Point out why certain algorithm choices (features, BA) matter.

4. **Week 11-12 (Nav2)**: Hands-on tuning; have students compete on navigation efficiency (path length, time, smoothness).

5. **Integration Projects**: Encourage students to combine all components—build full autonomy stack with simulated robot, implement SLAM + Nav2, measure success rate and path quality.

---

## Chapter Development Checklist

- [ ] Generate detailed worked examples with full code (Examples 1-3 above as starting point)
- [ ] Create visual diagrams/figures:
  - [Figure 2.1] System architecture (simulation → SLAM → Nav2)
  - [Figure 2.2] Camera projection geometry with distortion
  - [Figure 2.3] SLAM feature matching and bundle adjustment
  - [Figure 2.4] Costmap layers and inflation
  - [Figure 2.5] Behavior tree execution flow
- [ ] Prepare accompanying Jupyter notebooks demonstrating:
  - Gazebo world creation and sensor configuration
  - ORB feature detection and matching
  - Simple ICP algorithm (point cloud registration)
  - A* path planning
  - Nav2 configuration and tuning
- [ ] Create glossary entries for 30-40 key terms (physics, perception, navigation)
- [ ] Develop 15-20 end-of-chapter exercises (beginner, intermediate, advanced mix)
- [ ] Prepare lecture slides covering key concepts and algorithm walkthroughs
- [ ] Design assignments:
  - Lab 1: Simulate a mobile robot in Gazebo with sensors
  - Lab 2: Implement basic visual odometry
  - Lab 3: Run SLAM in simulation and evaluate map quality
  - Lab 4: Autonomous navigation challenge (obstacle course in simulation)
  - Final Project: End-to-end autonomous navigation system (simulation to real robot if available)

---

## Integration Notes for Textbook Author

This outline is structured to support a 3000-3500 word chapter that can be delivered in ~2 hours of reading plus ~3-4 hours of hands-on labs. The progression from fundamentals (physics simulation) through perception (vision/SLAM) to autonomy (navigation) mirrors typical roboticist problem-solving: build simulation, verify sensors, localize, plan and navigate.

**Key Success Criteria for Chapter Content**:
1. All learning outcomes verifiable through code/exercises
2. Equations/algorithms tie directly to code examples
3. Real-world applications (actual robots) motivate each section
4. Lab exercises self-contained and runnable in ROS 2 Docker environment
5. Glossary entries link back to precise definitions from content

