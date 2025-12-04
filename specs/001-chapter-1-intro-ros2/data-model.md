# Content Data Model: Chapter 1

**Date**: 2025-12-04
**Feature**: 001-chapter-1-intro-ros2
**Purpose**: Define content structure, learning objective mapping, and key entities

---

## Content Hierarchy & Structure

### Chapter-Level Organization

```
Chapter 1: Introduction to Physical AI & ROS 2 (3,000 words)
│
├── Introduction (Motivational hook - ~150 words)
│   └── Why embodied intelligence matters in robotics
│
├── 1.1 Foundations of Physical AI (~800 words)
│   ├── 1.1.1 What is Physical AI? (~150 words)
│   ├── 1.1.2 Embodied Intelligence (~150 words)
│   ├── 1.1.3 Digital AI vs. Physical AI vs. Embodied Intelligence (~200 words)
│   │   └── Table: ChatGPT, Tesla Autopilot, Boston Dynamics Atlas comparison
│   ├── 1.1.4 Real-World Constraints (~200 words)
│   │   └── LaTeX: g = 9.81 m/s², latency equations, friction models
│   └── 1.1.5 Why Physical AI Matters (~100 words)
│
├── 1.2 The Humanoid Robotics Landscape (~600 words)
│   ├── 1.2.1 Current State (~350 words)
│   │   ├── Boston Dynamics Atlas (100 words)
│   │   ├── Tesla Humanoid/Optimus (100 words)
│   │   └── Unitree G1 (150 words)
│   ├── 1.2.2 Sensor Systems (~150 words)
│   │   ├── LIDAR (passive vs. active)
│   │   ├── RGB Cameras
│   │   ├── Depth Cameras (RGB-D)
│   │   └── IMUs (Inertial Measurement Units)
│   └── 1.2.3 Degrees of Freedom & Applications (~100 words)
│
├── 1.3 The Robotic Nervous System: ROS 2 (~1,400 words)
│   ├── 1.3.1 ROS 2 Architecture Fundamentals (~200 words)
│   │   └── Diagram: Decoupled node architecture, pub/sub pattern
│   ├── 1.3.2 Nodes, Topics, and Pub/Sub Pattern (~350 words)
│   │   ├── Node lifecycle explanation
│   │   ├── Code Example 1: Publisher
│   │   ├── Code Example 2: Subscriber
│   │   └── Message flow diagram
│   ├── 1.3.3 Services and Actions (~250 words)
│   │   └── Table: Topics vs. Services vs. Actions comparison
│   ├── 1.3.4 Building ROS 2 Packages with Python (~300 words)
│   │   ├── Package structure overview
│   │   ├── Code Example 3: Launch file with parameter passing
│   │   └── ROS 2 parameter system
│   ├── 1.3.5 URDF: Robot Structure Description (~300 words)
│   │   ├── URDF XML elements explanation
│   │   ├── Code Example 4: 2-Link robotic arm URDF
│   │   └── Code Example 5: RViz launch file
│   └── 1.3.6 Launch Files and Multi-Node Systems (~150 words)
│       └── Orchestrating multiple nodes, real robot systems
│
├── 1.4 Summary and Key Takeaways (~200 words)
│   ├── Takeaway 1: Physical AI requires embodied systems
│   ├── Takeaway 2: Sensor-actuator loops enable real-time adaptation
│   ├── Takeaway 3: ROS 2 enables modular robot control
│   ├── Takeaway 4: URDF describes robot structure
│   ├── Takeaway 5: Python + ROS 2 enables rapid prototyping
│   ├── Takeaway 6: Real humanoid systems build on these concepts
│   └── Takeaway 7: Chapter 2 extends to simulation and perception
│
└── Further Reading & Resources
    └── Links to official docs, robotics papers, platforms
```

---

## Key Entities & Relationships

### 1. Physical AI (Concept)

**Definition**: AI systems operating in physical world under physical constraints

**Attributes**:
- **Embodiment**: Yes (has sensors and actuators)
- **Perception**: Real physical sensors (cameras, LIDAR, IMU, force sensors)
- **Action**: Physical actuators (motors, pneumatics, hydraulics)
- **Feedback Loop**: Real-time (milliseconds to seconds)
- **Constraints**: Gravity, friction, contact dynamics, energy budget, latency

**Learning Objective Mapping**:
- LO-1.1: Define Physical AI and distinguish from traditional AI
- LO-1.2: Explain embodied intelligence and its significance

**Introduced in**: Section 1.1.1-1.1.3
**Assessed in**: User Story 1 acceptance scenarios

---

### 2. Embodied Intelligence (Concept)

**Definition**: Intelligence arising from interaction between body morphology, sensors, and environment

**Key Principles**:
- **Morphological Computation**: Physical body structure reduces computational burden
- **Sensor-Actuator Loop**: Perception → Computation → Action → Environmental feedback
- **Tight Coupling**: Direct sensory feedback enables rapid environmental adaptation
- **Learning by Interaction**: Learns through physical experience, not offline datasets

**Examples**:
- Human hand dexterity: 23 DOF controlled by <1000 motor neurons (physical properties do much computation)
- Robot locomotion: Passive dynamics enable efficient bipedal walking
- Humanoid morphology: Human-like form enables natural interaction in human spaces

**Learning Objective Mapping**:
- LO-1.2: Explain embodied intelligence significance

**Introduced in**: Section 1.1.2-1.1.4
**Assessed in**: User Story 1 acceptance scenarios

---

### 3. Humanoid Platform (Entity Class)

**Attributes**:
- `name`: String (e.g., "Boston Dynamics Atlas")
- `manufacturer`: String
- `status`: Enum (Research, Prototype, Commercial)
- `height_m`: Float (meters)
- `dof_count`: Integer (degrees of freedom)
- `primary_sensors`: List of Sensor enums
- `primary_applications`: List of String
- `year_available`: Integer

**Instances**:
1. Boston Dynamics Atlas
   - status: Research
   - dof_count: 28
   - primary_sensors: [LIDAR, Stereo_Camera, IMU, ForceTorque]
   - primary_applications: ["Research", "Disaster_Response", "Inspection"]

2. Tesla Optimus
   - status: Prototype
   - dof_count: 40+
   - primary_sensors: [RGB_Camera, Depth_Sensor, Proprioceptive_Sensors]
   - primary_applications: ["Manufacturing", "General_Purpose"]

3. Unitree G1
   - status: Commercial
   - dof_count: 23
   - primary_sensors: [RGB_Camera, Depth_Camera, IMU, Force_Sensors]
   - primary_applications: ["Education", "Research", "Commercial"]

**Learning Objective Mapping**:
- LO-1.1: Understand current state of humanoid robotics

**Introduced in**: Section 1.2
**Assessed in**: User Story 2 acceptance scenarios

---

### 4. Sensor (Entity Class)

**Attributes**:
- `name`: String
- `type`: Enum (Passive_Vision, Active_Vision, Inertial, Force, Proprioceptive)
- `output`: String (description of measurement output)
- `use_cases`: List of String

**Instances**:

#### RGB Camera (Passive Vision)
- type: Passive_Vision
- output: RGB images, visual feature detection
- use_cases: ["Object recognition", "Visual servoing", "Scene understanding"]

#### Depth Camera (Active Vision)
- type: Active_Vision
- output: RGB + depth (3D point clouds)
- use_cases: ["Object grasping", "Obstacle avoidance", "Surface reconstruction"]

#### LIDAR (Active Vision)
- type: Active_Vision
- output: 3D point cloud, distance map
- use_cases: ["Outdoor navigation", "Mapping", "Obstacle detection"]

#### IMU (Inertial)
- type: Inertial
- output: Acceleration, angular velocity, orientation
- use_cases: ["Balance control", "Fall detection", "Orientation tracking"]

#### Force/Torque Sensor
- type: Force
- output: Forces and torques in 6D (Fx, Fy, Fz, Tx, Ty, Tz)
- use_cases: ["Manipulation feedback", "Contact detection", "Impedance control"]

**Learning Objective Mapping**:
- LO-1.1: Understand sensor-actuator loops

**Introduced in**: Section 1.2.2
**Assessed in**: User Story 2 acceptance scenarios

---

### 5. ROS 2 Node (Architecture Component)

**Definition**: Independent ROS 2 process with its own lifecycle, publishers, subscribers, services

**Attributes**:
- `name`: String (node name, e.g., "sensor_reader", "control_node")
- `publishers`: List of Topic subscriptions this node publishes to
- `subscribers`: List of Topic subscriptions this node subscribes from
- `services`: List of Service this node provides
- `state`: Enum (Created, Configured, Active, Finalized)

**Relationships**:
- Publishes to: Topic (0..*)
- Subscribes from: Topic (0..*)
- Provides: Service (0..*)
- Calls: Service (0..*)

**Learning Objective Mapping**:
- LO-1.3: Understand ROS 2 nodes, topics, pub/sub

**Introduced in**: Section 1.3.1-1.3.2
**Assessed in**: User Story 3 & 4 acceptance scenarios

---

### 6. Topic (Communication Channel)

**Definition**: Named asynchronous communication channel for pub/sub messaging

**Attributes**:
- `name`: String (e.g., "/sensor_data", "/motor_commands")
- `message_type`: String (e.g., "std_msgs/String", "sensor_msgs/PointCloud2")
- `publishers`: Set of Node
- `subscribers`: Set of Node
- `qos_profile`: QoS (queue depth, reliability, etc.)

**Relationships**:
- Published by: Node (1..*)
- Subscribed by: Node (1..*)

**Use Cases** (from Table 1.3.3):
- High-frequency sensor data: `/camera/image`, `/lidar/pointcloud`, `/imu/data`
- Low-frequency commands: `/motion/command`, `/gripper/control`

**Learning Objective Mapping**:
- LO-1.3: Understand ROS 2 pub/sub communication

**Introduced in**: Section 1.3.2
**Assessed in**: User Story 3 acceptance scenarios

---

### 7. Service (Request-Reply Pattern)

**Definition**: Synchronous request-reply communication pattern for RPC-style interactions

**Attributes**:
- `name`: String (e.g., "/robot/get_position")
- `request_type`: Message struct
- `response_type`: Message struct

**Use Cases** (from Table 1.3.3):
- Query operations: "Get current joint positions"
- Configuration changes: "Set parameter X to value Y"
- One-time computations: "Calculate inverse kinematics"

**Learning Objective Mapping**:
- LO-1.3: Understand when to use services vs. topics

**Introduced in**: Section 1.3.3
**Assessed in**: User Story 3 acceptance scenarios

---

### 8. Action (Asynchronous Task Pattern)

**Definition**: Asynchronous communication pattern for long-running tasks with feedback

**Attributes**:
- `name`: String (e.g., "/robot/move_to_position")
- `goal`: Message struct (desired state)
- `feedback`: Message struct (progress updates)
- `result`: Message struct (final outcome)
- `state`: Enum (Active, Succeeded, Failed, Canceled)

**Use Cases** (from Table 1.3.3):
- Navigation: "Move robot to XY position" (goal) with periodic position updates (feedback)
- Manipulation: "Grasp object" (goal) with force feedback (feedback)
- Trajectory execution: "Follow path" (goal) with progress percentage (feedback)

**Learning Objective Mapping**:
- LO-1.3: Understand when to use actions for long-running tasks

**Introduced in**: Section 1.3.3
**Assessed in**: User Story 3 acceptance scenarios

---

### 9. URDF Robot Description (XML Structure)

**Definition**: Unified Robot Description Format (URDF) - XML-based format describing robot links, joints, geometry, kinematics

**Key Elements**:

#### Link Element
```xml
<link name="link_name">
  <visual>
    <geometry><!-- Box, Cylinder, Sphere, Mesh --></geometry>
    <material><!-- Color definition --></material>
  </visual>
  <collision>
    <geometry><!-- Same as visual, for physics --></geometry>
  </collision>
  <inertial><!-- Mass and inertia tensor --></inertial>
</link>
```

**Attributes**:
- `name`: Link identifier
- `visual_geometry`: For rendering in RViz
- `collision_geometry`: For physics simulation (may differ from visual)
- `mass`: For dynamics calculations
- `inertia_tensor`: Resistance to rotation

#### Joint Element
```xml
<joint name="joint_name" type="revolute|prismatic|fixed">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="0 0 1"/><!-- Joint axis -->
  <limit lower="-1.57" upper="1.57"/><!-- Joint limits -->
</joint>
```

**Attributes**:
- `name`: Joint identifier
- `type`: Revolute (rotation), Prismatic (translation), Fixed (no motion)
- `parent`: Link this joint connects from
- `child`: Link this joint connects to
- `origin`: Transformation from parent to joint frame
- `axis`: Direction of rotation/translation
- `limits`: Position constraints (min/max angles or distances)

**Learning Objective Mapping**:
- LO-1.5: Describe robot structure using URDF

**Introduced in**: Section 1.3.5
**Assessed in**: User Story 5 acceptance scenarios

---

### 10. 2-Link Robotic Arm (URDF Instantiation)

**Concrete Example**: 2-link planar manipulator arm

**Structure**:
```
World Frame (Fixed)
    ↓
base_link (rectangular base, 1 kg)
    ↓ [Joint 1: Revolute, Z-axis, ±π radians]
link_1 (upper arm, cylinder 0.5m long, 1 kg)
    ↓ [Joint 2: Revolute, Z-axis, ±π/2 to +π radians]
link_2 (forearm, cylinder 0.4m long, 0.5 kg)
```

**URDF Features Demonstrated**:
- Link definitions with visual and collision geometry
- Revolute joint with realistic limits
- Coordinate frame transformations (origin elements)
- Mass and inertia for dynamics

**Learning Objective Mapping**:
- LO-1.5: Write and modify URDF, visualize in RViz

**Introduced in**: Section 1.3.5 (Code Example 4)
**Assessed in**: User Story 5 acceptance scenarios

---

## Progressive Disclosure Strategy

### Learning Path

1. **Conceptual Foundation** (Section 1.1)
   - Introduce Physical AI concept
   - Establish why embodied intelligence matters
   - Motivate need for real-time perception-action loops

2. **Real-World Context** (Section 1.2)
   - Concrete examples of current humanoid platforms
   - Identify sensor types and their roles
   - Connect abstraction to concrete hardware

3. **Technical Implementation** (Section 1.3)
   - ROS 2 as the middleware enabling modular robotics
   - Specific communication patterns (pub/sub, services, actions)
   - Hands-on code examples students can run

4. **Synthesis** (Section 1.4)
   - Connect Physical AI concepts to ROS 2 implementation
   - Summarize key takeaways
   - Prepare for Chapter 2 (simulation, perception, control)

### Bloom's Taxonomy Mapping

| Level | Learning Activities | Chapter 1 Coverage |
|-------|-------------------|-------------------|
| **Remember** | Recall Physical AI definition | Section 1.1.1 |
| **Understand** | Explain embodied intelligence | Section 1.1.2-1.1.4 |
| **Apply** | Write ROS 2 publisher node | Section 1.3.2, User Story 4 |
| **Analyze** | Compare platforms and sensors | Section 1.2, User Story 2 |
| **Evaluate** | Choose communication pattern | Section 1.3.3, User Story 3 |
| **Create** | Write URDF for custom arm | Section 1.3.5, User Story 5 |

---

## Concept Sequencing Rules

1. **Define before use**: All key terms defined in body text before first use
2. **Simple to complex**: Conceptual foundations (Physical AI) before technical implementations (ROS 2)
3. **Concrete to abstract**: Real robot examples (Atlas, Optimus) before abstract patterns (nodes, topics)
4. **Theory to practice**: Concept explanation followed immediately by code example
5. **Glossary terms**: Key robotics terms identified for cross-reference

---

## Content Dependencies

- **Section 1.1** is prerequisite for understanding **why** ROS 2 is needed
- **Section 1.2** provides concrete context for **where** ROS 2 is applied
- **Section 1.3** builds on **Section 1.1-1.2** understanding
- **Code Examples 1-3** (ROS 2) depend on **understanding pub/sub pattern** from text
- **Code Example 4** (URDF) demonstrates structure described in text
- **Section 1.4** synthesizes all previous concepts

---

## Next Steps

1. Use this data model to structure textbook-author prompt
2. Ensure content follows progressive disclosure strategy
3. Verify all code examples address their learning objectives
4. Cross-check that all entities and relationships are properly introduced and assessed

