# Tasks: Comprehensive Glossary for Robotics Textbook

**Feature Branch**: `002-textbook-glossary` | **Date**: 2025-12-04 | **Plan**: [plan.md](plan.md) | **Spec**: [spec.md](spec.md)

---

## Implementation Strategy

**MVP Scope**: User Story 1 (P1) - Create and organize 60-80 glossary terms with definitions and cross-references
**Incremental Delivery**: P2 (search/filtering) and P3 (chapter linking) are enhancements after MVP foundation
**Parallel Opportunities**: Term creation is parallelizable; multiple contributors can work on different term files simultaneously

---

## Phase 1: Setup & Infrastructure

Initialize Docusaurus glossary structure and configuration.

- [ ] T001 Create `/frontend/docs/glossary/` directory structure per plan.md project structure
- [ ] T002 Create `/frontend/docs/glossary/_category_.json` with glossary section metadata and sidebar position
- [ ] T003 Create `/frontend/docs/glossary/index.md` glossary home page with description and search UI placeholder
- [ ] T004 Create `/frontend/docs/glossary/01-introduction.md` glossary usage guide and term contribution instructions
- [ ] T005 Create `/frontend/docs/glossary/terms/` subdirectory for individual term markdown files
- [ ] T006 Update `/frontend/docusaurus.config.js` sidebar configuration to include glossary section
- [ ] T007 Update `/frontend/docs/sidebar.ts` to add glossary navigation entry with position (after chapters, before other docs)
- [ ] T008 [P] Configure Docusaurus search settings in `docusaurus.config.js` to enable full-text search of glossary terms

---

## Phase 2: Foundational - Glossary Categories & Validation

Establish approved categories and validation framework before term creation.

- [ ] T009 Create glossary category reference document `/frontend/docs/glossary/categories.md` listing 8 approved categories:
  - ROS 2 Architecture
  - Simulation & Physics
  - Perception & Sensing
  - Kinematics & Dynamics
  - Control & Learning
  - Software Concepts
  - Software Tools
  - Hardware Components
- [ ] T010 Create link validation script `/frontend/scripts/validate-glossary-links.js` to:
  - Count total terms in `/docs/glossary/terms/`
  - Verify each term has valid YAML frontmatter (id, term, categories, related_terms, chapter_introduced, section_reference)
  - Check all related_terms references exist (cross-reference validation)
  - Report broken links and invalid category assignments
- [ ] T011 Create term count validation script `/frontend/scripts/validate-term-count.js` to:
  - Count term files in `/docs/glossary/terms/`
  - Verify count is between 60-80
  - Output count and filename list for verification
- [ ] T012 Create category consistency validator `/frontend/scripts/validate-categories.js` to:
  - Verify each term has 1-3 categories from approved list
  - Report terms with zero or >3 categories
  - Report undefined category names
- [ ] T013 [P] Add validation scripts to `package.json` under scripts section:
  - `npm run glossary:validate-links`
  - `npm run glossary:validate-terms`
  - `npm run glossary:validate-categories`
  - `npm run glossary:validate-all` (runs all three)
- [ ] T014 Add pre-build hook to `docusaurus.config.js` to run validation scripts before build

---

## Phase 3: User Story 1 - Create Glossary Terms (P1)

**Goal**: Extract 60-80 technical terms from 3 chapters, create markdown files with definitions, establish cross-references

**Independent Test**:
- `npm run glossary:validate-all` passes
- Glossary page loads in <2s
- All 60-80 terms visible on /docs/glossary/index.md with links to term pages

### Chapter 1 Terms (~25 terms from "Introduction to Physical AI & ROS 2")

- [ ] T015 [P] [US1] Create term file `/frontend/docs/glossary/terms/01-embodied-intelligence.md` for "Embodied Intelligence"
  - Acronym: None
  - Categories: ["Software Concepts", "ROS 2 Architecture"]
  - Definition: ~100 words on embodied AI in robotics
  - Related: ["physical-ai", "perception", "sensor-fusion"]
  - Chapter: "Chapter 1", Section: "1.0: What is Physical AI?"
- [ ] T016 [P] [US1] Create term file `/frontend/docs/glossary/terms/02-physical-ai.md` for "Physical AI"
  - Acronym: None
  - Categories: ["Software Concepts"]
  - Definition: ~120 words on Physical AI definition and importance
  - Related: ["embodied-intelligence", "humanoid-robotics", "ros-2"]
  - Chapter: "Chapter 1", Section: "1.0: What is Physical AI?"
- [ ] T017 [P] [US1] Create term file `/frontend/docs/glossary/terms/03-ros-2.md` for "ROS 2"
  - Acronym: "Robot Operating System 2"
  - Categories: ["ROS 2 Architecture", "Software Tools"]
  - Definition: ~150 words on ROS 2 middleware, publish-subscribe, nodes
  - Related: ["node", "topic", "publisher-subscriber", "launch-file", "rclpy"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T018 [P] [US1] Create term file `/frontend/docs/glossary/terms/04-node.md` for "Node"
  - Acronym: None
  - Categories: ["ROS 2 Architecture"]
  - Definition: ~100 words on ROS 2 nodes as independent processes
  - Related: ["ros-2", "topic", "publisher-subscriber", "service", "action"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T019 [P] [US1] Create term file `/frontend/docs/glossary/terms/05-topic.md` for "Topic"
  - Acronym: None
  - Categories: ["ROS 2 Architecture"]
  - Definition: ~100 words on ROS 2 topics for pub-sub communication
  - Related: ["ros-2", "publisher-subscriber", "node", "message"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T020 [P] [US1] Create term file `/frontend/docs/glossary/terms/06-publisher-subscriber.md` for "Publisher-Subscriber Pattern"
  - Acronym: "Pub-Sub"
  - Categories: ["ROS 2 Architecture", "Software Concepts"]
  - Definition: ~120 words on decoupled communication via pub-sub
  - Related: ["ros-2", "topic", "message", "node"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T021 [P] [US1] Create term file `/frontend/docs/glossary/terms/07-service.md` for "Service"
  - Acronym: None
  - Categories: ["ROS 2 Architecture"]
  - Definition: ~100 words on synchronous request-response in ROS 2
  - Related: ["ros-2", "action", "topic", "client-server"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T022 [P] [US1] Create term file `/frontend/docs/glossary/terms/08-action.md` for "Action"
  - Acronym: None
  - Categories: ["ROS 2 Architecture"]
  - Definition: ~110 words on asynchronous goal-based communication with feedback
  - Related: ["ros-2", "service", "goal", "feedback"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T023 [P] [US1] Create term file `/frontend/docs/glossary/terms/09-urdf.md` for "URDF"
  - Acronym: "Unified Robot Description Format"
  - Categories: ["ROS 2 Architecture", "Hardware Components"]
  - Definition: ~120 words on URDF XML format for robot structure
  - Related: ["ros-2", "link", "joint", "robot-model"]
  - Chapter: "Chapter 1", Section: "1.2: URDF and Robot Models"
- [ ] T024 [P] [US1] Create term file `/frontend/docs/glossary/terms/10-link.md` for "Link"
  - Acronym: None
  - Categories: ["Hardware Components"]
  - Definition: ~100 words on URDF links as rigid bodies
  - Related: ["urdf", "joint", "robot-model", "collision-geometry"]
  - Chapter: "Chapter 1", Section: "1.2: URDF and Robot Models"
- [ ] T025 [P] [US1] Create term file `/frontend/docs/glossary/terms/11-joint.md` for "Joint"
  - Acronym: None
  - Categories: ["Hardware Components", "Kinematics & Dynamics"]
  - Definition: ~110 words on URDF joints connecting links
  - Related: ["urdf", "link", "dof", "kinematics"]
  - Chapter: "Chapter 1", Section: "1.2: URDF and Robot Models"
- [ ] T026 [P] [US1] Create term file `/frontend/docs/glossary/terms/12-dof.md` for "Degrees of Freedom"
  - Acronym: "DOF"
  - Categories: ["Kinematics & Dynamics", "Hardware Components"]
  - Definition: ~100 words on DOF in robotics
  - Related: ["joint", "kinematics", "robot-configuration"]
  - Chapter: "Chapter 1", Section: "1.2: URDF and Robot Models"
- [ ] T027 [P] [US1] Create term file `/frontend/docs/glossary/terms/13-end-effector.md` for "End-Effector"
  - Acronym: None
  - Categories: ["Hardware Components", "Kinematics & Dynamics"]
  - Definition: ~110 words on end-effector as robot's tool/gripper
  - Related: ["kinematics", "forward-kinematics", "inverse-kinematics"]
  - Chapter: "Chapter 1", Section: "1.3: Humanoid Robot Anatomy"
- [ ] T028 [P] [US1] Create term file `/frontend/docs/glossary/terms/14-lidar.md` for "LIDAR"
  - Acronym: "Light Detection and Ranging"
  - Categories: ["Perception & Sensing", "Hardware Components"]
  - Definition: ~120 words on LIDAR sensors for 3D perception
  - Related: ["sensor-fusion", "point-cloud", "perception", "rgb-d-camera"]
  - Chapter: "Chapter 1", Section: "1.3: Sensors and Perception"
- [ ] T029 [P] [US1] Create term file `/frontend/docs/glossary/terms/15-imu.md` for "IMU"
  - Acronym: "Inertial Measurement Unit"
  - Categories: ["Perception & Sensing", "Hardware Components"]
  - Definition: ~110 words on IMU for motion and acceleration measurement
  - Related: ["sensor-fusion", "accelerometer", "gyroscope", "perception"]
  - Chapter: "Chapter 1", Section: "1.3: Sensors and Perception"
- [ ] T030 [P] [US1] Create term file `/frontend/docs/glossary/terms/16-sensor-fusion.md` for "Sensor Fusion"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on combining multiple sensor inputs for robust perception
  - Related: ["lidar", "imu", "rgb-d-camera", "kalman-filter", "perception"]
  - Chapter: "Chapter 1", Section: "1.3: Sensors and Perception"
- [ ] T031 [P] [US1] Create term file `/frontend/docs/glossary/terms/17-rclpy.md` for "rclpy"
  - Acronym: "ROS 2 Client Library for Python"
  - Categories: ["ROS 2 Architecture", "Software Tools"]
  - Definition: ~110 words on rclpy Python library for ROS 2 programming
  - Related: ["ros-2", "node", "client-library"]
  - Chapter: "Chapter 1", Section: "1.4: ROS 2 Programming Basics"
- [ ] T032 [P] [US1] Create term file `/frontend/docs/glossary/terms/18-colcon.md` for "colcon"
  - Acronym: "Collective Construction"
  - Categories: ["ROS 2 Architecture", "Software Tools"]
  - Definition: ~110 words on colcon build tool for ROS 2
  - Related: ["ros-2", "package", "build-system"]
  - Chapter: "Chapter 1", Section: "1.4: ROS 2 Programming Basics"
- [ ] T033 [P] [US1] Create term file `/frontend/docs/glossary/terms/19-launch-file.md` for "Launch File"
  - Acronym: None
  - Categories: ["ROS 2 Architecture", "Software Tools"]
  - Definition: ~120 words on ROS 2 launch files for configuring multiple nodes
  - Related: ["ros-2", "node", "configuration"]
  - Chapter: "Chapter 1", Section: "1.4: ROS 2 Programming Basics"
- [ ] T034 [P] [US1] Create term file `/frontend/docs/glossary/terms/20-message.md` for "Message"
  - Acronym: None
  - Categories: ["ROS 2 Architecture"]
  - Definition: ~100 words on ROS 2 messages as data structures
  - Related: ["ros-2", "topic", "service", "publisher-subscriber"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"
- [ ] T035 [P] [US1] Create term file `/frontend/docs/glossary/terms/21-client-server.md` for "Client-Server"
  - Acronym: None
  - Categories: ["ROS 2 Architecture", "Software Concepts"]
  - Definition: ~100 words on client-server communication pattern
  - Related: ["service", "request-response"]
  - Chapter: "Chapter 1", Section: "1.1: Introduction to ROS 2"

### Chapter 2 Terms (~25 terms from "Robot Simulation & AI Perception")

- [ ] T036 [P] [US1] Create term file `/frontend/docs/glossary/terms/22-gazebo.md` for "Gazebo"
  - Acronym: None
  - Categories: ["Simulation & Physics", "Software Tools"]
  - Definition: ~130 words on Gazebo robot simulator for testing algorithms
  - Related: ["physics-engine", "sdf", "sim-to-real", "nvidia-isaac-sim"]
  - Chapter: "Chapter 2", Section: "2.1: Robot Simulation with Gazebo"
- [ ] T037 [P] [US1] Create term file `/frontend/docs/glossary/terms/23-sdf.md` for "SDF"
  - Acronym: "Simulation Description Format"
  - Categories: ["Simulation & Physics", "Software Tools"]
  - Definition: ~120 words on SDF XML format for simulation models
  - Related: ["gazebo", "urdf", "physics-engine"]
  - Chapter: "Chapter 2", Section: "2.1: Robot Simulation with Gazebo"
- [ ] T038 [P] [US1] Create term file `/frontend/docs/glossary/terms/24-physics-engine.md` for "Physics Engine"
  - Acronym: None
  - Categories: ["Simulation & Physics"]
  - Definition: ~120 words on physics simulation engines for realistic dynamics
  - Related: ["gazebo", "sdf", "dynamics", "collision-detection"]
  - Chapter: "Chapter 2", Section: "2.1: Robot Simulation with Gazebo"
- [ ] T039 [P] [US1] Create term file `/frontend/docs/glossary/terms/25-point-cloud.md` for "Point Cloud"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on 3D point clouds from LIDAR/RGB-D sensors
  - Related: ["lidar", "rgb-d-camera", "3d-perception", "pcl"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Point Clouds"
- [ ] T040 [P] [US1] Create term file `/frontend/docs/glossary/terms/26-rgb-d-camera.md` for "RGB-D Camera"
  - Acronym: "Red-Green-Blue-Depth"
  - Categories: ["Perception & Sensing", "Hardware Components"]
  - Definition: ~120 words on RGB-D sensors providing color and depth data
  - Related: ["point-cloud", "depth-map", "lidar", "3d-perception"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Point Clouds"
- [ ] T041 [P] [US1] Create term file `/frontend/docs/glossary/terms/27-slam.md` for "SLAM"
  - Acronym: "Simultaneous Localization and Mapping"
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~130 words on SLAM for robot navigation and map building
  - Related: ["vslam", "localization", "mapping", "nav2"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"
- [ ] T042 [P] [US1] Create term file `/frontend/docs/glossary/terms/28-vslam.md` for "VSLAM"
  - Acronym: "Visual SLAM"
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on vision-based SLAM using camera images
  - Related: ["slam", "visual-odometry", "feature-detection"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"
- [ ] T043 [P] [US1] Create term file `/frontend/docs/glossary/terms/29-nvidia-isaac-sim.md` for "NVIDIA Isaac Sim"
  - Acronym: None
  - Categories: ["Simulation & Physics", "Software Tools"]
  - Definition: ~120 words on NVIDIA Isaac Sim for photorealistic robot simulation
  - Related: ["gazebo", "sim-to-real", "synthetic-data", "nvidia-omniverse"]
  - Chapter: "Chapter 2", Section: "2.1: Advanced Simulation with Isaac Sim"
- [ ] T044 [P] [US1] Create term file `/frontend/docs/glossary/terms/30-omniverse-usd.md` for "Omniverse USD"
  - Acronym: "Universal Scene Description"
  - Categories: ["Simulation & Physics", "Software Tools"]
  - Definition: ~120 words on USD format for 3D scene representation
  - Related: ["nvidia-isaac-sim", "nvidia-omniverse", "3d-modeling"]
  - Chapter: "Chapter 2", Section: "2.1: Advanced Simulation with Isaac Sim"
- [ ] T045 [P] [US1] Create term file `/frontend/docs/glossary/terms/31-synthetic-data.md` for "Synthetic Data"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on computer-generated data for training perception models
  - Related: ["sim-to-real", "nvidia-isaac-sim", "training-data"]
  - Chapter: "Chapter 2", Section: "2.4: Synthetic Data Generation"
- [ ] T046 [P] [US1] Create term file `/frontend/docs/glossary/terms/32-sim-to-real.md` for "Sim-to-Real"
  - Acronym: None
  - Categories: ["Simulation & Physics", "Software Concepts"]
  - Definition: ~130 words on transferring policies from simulation to real robots
  - Related: ["simulation", "synthetic-data", "domain-adaptation"]
  - Chapter: "Chapter 2", Section: "2.4: Sim-to-Real Transfer"
- [ ] T047 [P] [US1] Create term file `/frontend/docs/glossary/terms/33-nav2.md` for "Nav2"
  - Acronym: "Navigation 2"
  - Categories: ["Software Tools", "Software Concepts"]
  - Definition: ~130 words on Nav2 ROS 2 navigation stack for autonomous movement
  - Related: ["ros-2", "slam", "path-planning", "costmap"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"
- [ ] T048 [P] [US1] Create term file `/frontend/docs/glossary/terms/34-costmap.md` for "Costmap"
  - Acronym: "Cost Map"
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on costmaps for obstacle representation in navigation
  - Related: ["nav2", "path-planning", "obstacle-detection"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning and Navigation"
- [ ] T049 [P] [US1] Create term file `/frontend/docs/glossary/terms/35-path-planning.md` for "Path Planning"
  - Acronym: None
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~130 words on algorithms for planning robot trajectories
  - Related: ["nav2", "a-star", "rrt", "motion-planning"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning and Navigation"
- [ ] T050 [P] [US1] Create term file `/frontend/docs/glossary/terms/36-a-star.md` for "A* Algorithm"
  - Acronym: "A-Star"
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~120 words on A* search algorithm for optimal path planning
  - Related: ["path-planning", "dijkstra", "heuristic-search"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning Algorithms"
- [ ] T051 [P] [US1] Create term file `/frontend/docs/glossary/terms/37-behavior-tree.md` for "Behavior Tree"
  - Acronym: None
  - Categories: ["Control & Learning", "Software Concepts"]
  - Definition: ~130 words on behavior trees for modular robot control
  - Related: ["state-machine", "task-planning", "decision-making"]
  - Chapter: "Chapter 2", Section: "2.5: Behavior Trees and Task Planning"
- [ ] T052 [P] [US1] Create term file `/frontend/docs/glossary/terms/38-3d-perception.md` for "3D Perception"
  - Acronym: None
  - Categories: ["Perception & Sensing"]
  - Definition: ~120 words on 3D perception from multiple sensors
  - Related: ["point-cloud", "lidar", "rgb-d-camera", "sensor-fusion"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Point Clouds"
- [ ] T053 [P] [US1] Create term file `/frontend/docs/glossary/terms/39-depth-map.md` for "Depth Map"
  - Acronym: None
  - Categories: ["Perception & Sensing"]
  - Definition: ~110 words on depth maps from depth sensors
  - Related: ["rgb-d-camera", "point-cloud", "stereo-vision"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Point Clouds"
- [ ] T054 [P] [US1] Create term file `/frontend/docs/glossary/terms/40-feature-detection.md` for "Feature Detection"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on detecting distinctive features in images
  - Related: ["vslam", "visual-odometry", "image-processing"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"
- [ ] T055 [P] [US1] Create term file `/frontend/docs/glossary/terms/41-localization.md` for "Localization"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on determining robot position in environment
  - Related: ["slam", "mapping", "gps", "odometry"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"
- [ ] T056 [P] [US1] Create term file `/frontend/docs/glossary/terms/42-mapping.md` for "Mapping"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on building environment maps
  - Related: ["slam", "localization", "occupancy-grid"]
  - Chapter: "Chapter 2", Section: "2.3: SLAM and Navigation"

### Chapter 3 Terms (~25 terms from "Humanoid Robot Control & Intelligence")

- [ ] T057 [P] [US1] Create term file `/frontend/docs/glossary/terms/43-forward-kinematics.md` for "Forward Kinematics"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~120 words on calculating end-effector position from joint angles
  - Related: ["inverse-kinematics", "jacobian", "kinematics"]
  - Chapter: "Chapter 3", Section: "3.1: Forward and Inverse Kinematics"
- [ ] T058 [P] [US1] Create term file `/frontend/docs/glossary/terms/44-inverse-kinematics.md` for "Inverse Kinematics"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~130 words on calculating joint angles for target end-effector position
  - Related: ["forward-kinematics", "jacobian", "ik-solver"]
  - Chapter: "Chapter 3", Section: "3.1: Forward and Inverse Kinematics"
- [ ] T059 [P] [US1] Create term file `/frontend/docs/glossary/terms/45-jacobian.md` for "Jacobian Matrix"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~130 words on Jacobian relating joint velocities to end-effector velocities
  - Related: ["forward-kinematics", "inverse-kinematics", "differential-kinematics"]
  - Chapter: "Chapter 3", Section: "3.1: Kinematics Fundamentals"
- [ ] T060 [P] [US1] Create term file `/frontend/docs/glossary/terms/46-zmp.md` for "Zero Moment Point"
  - Acronym: "ZMP"
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~120 words on ZMP for bipedal balance and gait stability
  - Related: ["bipedal-locomotion", "gait", "balance-control"]
  - Chapter: "Chapter 3", Section: "3.2: Bipedal Locomotion and Gait"
- [ ] T061 [P] [US1] Create term file `/frontend/docs/glossary/terms/47-bipedal-locomotion.md` for "Bipedal Locomotion"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~130 words on two-legged walking and balance control
  - Related: ["gait", "zmp", "balance-control", "humanoid-robotics"]
  - Chapter: "Chapter 3", Section: "3.2: Bipedal Locomotion and Gait"
- [ ] T062 [P] [US1] Create term file `/frontend/docs/glossary/terms/48-gait.md` for "Gait"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~120 words on walking patterns and gait analysis
  - Related: ["bipedal-locomotion", "zmp", "motion-planning"]
  - Chapter: "Chapter 3", Section: "3.2: Bipedal Locomotion and Gait"
- [ ] T063 [P] [US1] Create term file `/frontend/docs/glossary/terms/49-grasp-planning.md` for "Grasp Planning"
  - Acronym: None
  - Categories: ["Control & Learning", "Kinematics & Dynamics"]
  - Definition: ~130 words on planning how to grasp objects with manipulator
  - Related: ["inverse-kinematics", "end-effector", "manipulation"]
  - Chapter: "Chapter 3", Section: "3.3: Manipulation and Grasping"
- [ ] T064 [P] [US1] Create term file `/frontend/docs/glossary/terms/50-hri.md` for "Human-Robot Interaction"
  - Acronym: "HRI"
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~130 words on interaction between humans and robots
  - Related: ["gesture-recognition", "natural-language-processing", "safety"]
  - Chapter: "Chapter 3", Section: "3.4: Human-Robot Interaction"
- [ ] T065 [P] [US1] Create term file `/frontend/docs/glossary/terms/51-openai-whisper.md` for "OpenAI Whisper"
  - Acronym: None
  - Categories: ["Software Tools", "Software Concepts"]
  - Definition: ~120 words on Whisper speech recognition model
  - Related: ["speech-recognition", "nlp", "language-understanding"]
  - Chapter: "Chapter 3", Section: "3.4: Speech Recognition and NLP"
- [ ] T066 [P] [US1] Create term file `/frontend/docs/glossary/terms/52-llm.md` for "Large Language Model"
  - Acronym: "LLM"
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~130 words on LLMs for language understanding and generation
  - Related: ["natural-language-processing", "transformer", "openai-gpt"]
  - Chapter: "Chapter 3", Section: "3.4: Language Understanding and Generation"
- [ ] T067 [P] [US1] Create term file `/frontend/docs/glossary/terms/53-cognitive-planning.md` for "Cognitive Planning"
  - Acronym: None
  - Categories: ["Control & Learning", "Software Concepts"]
  - Definition: ~130 words on high-level task planning and reasoning
  - Related: ["task-planning", "llm", "behavior-tree", "knowledge-representation"]
  - Chapter: "Chapter 3", Section: "3.5: Cognitive Control and Planning"
- [ ] T068 [P] [US1] Create term file `/frontend/docs/glossary/terms/54-action-primitive.md` for "Action Primitive"
  - Acronym: None
  - Categories: ["Control & Learning", "Software Concepts"]
  - Definition: ~120 words on basic building blocks of robot actions
  - Related: ["motion-primitive", "behavior-tree", "task-planning"]
  - Chapter: "Chapter 3", Section: "3.5: Cognitive Control and Planning"
- [ ] T069 [P] [US1] Create term file `/frontend/docs/glossary/terms/55-vla.md` for "Vision-Language-Action"
  - Acronym: "VLA"
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~130 words on VLA models integrating vision, language, and action
  - Related: ["multimodal-learning", "llm", "visual-perception", "robotics"]
  - Chapter: "Chapter 3", Section: "3.5: Vision-Language-Action Models"
- [ ] T070 [P] [US1] Create term file `/frontend/docs/glossary/terms/56-yolo.md` for "YOLO"
  - Acronym: "You Only Look Once"
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on YOLO real-time object detection
  - Related: ["object-detection", "neural-network", "real-time-perception"]
  - Chapter: "Chapter 3", Section: "3.3: Object Detection and Recognition"
- [ ] T071 [P] [US1] Create term file `/frontend/docs/glossary/terms/57-object-detection.md` for "Object Detection"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on detecting and localizing objects in images
  - Related: ["yolo", "cnn", "image-segmentation"]
  - Chapter: "Chapter 3", Section: "3.3: Object Detection and Recognition"
- [ ] T072 [P] [US1] Create term file `/frontend/docs/glossary/terms/58-semantic-segmentation.md` for "Semantic Segmentation"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~130 words on pixel-level classification of image regions
  - Related: ["object-detection", "image-segmentation", "cnn"]
  - Chapter: "Chapter 3", Section: "3.3: Object Detection and Recognition"
- [ ] T073 [P] [US1] Create term file `/frontend/docs/glossary/terms/59-humanoid-robotics.md` for "Humanoid Robotics"
  - Acronym: None
  - Categories: ["Hardware Components", "Software Concepts"]
  - Definition: ~130 words on designing and controlling human-shaped robots
  - Related: ["bipedal-locomotion", "manipulation", "hri"]
  - Chapter: "Chapter 3", Section: "1.0: Humanoid Robot Design"
- [ ] T074 [P] [US1] Create term file `/frontend/docs/glossary/terms/60-transformer.md` for "Transformer"
  - Acronym: None
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~120 words on transformer neural network architecture
  - Related: ["llm", "attention-mechanism", "deep-learning"]
  - Chapter: "Chapter 3", Section: "3.4: Deep Learning for Robotics"
- [ ] T075 [P] [US1] Create term file `/frontend/docs/glossary/terms/61-natural-language-processing.md` for "Natural Language Processing"
  - Acronym: "NLP"
  - Categories: ["Software Concepts", "Control & Learning"]
  - Definition: ~130 words on processing and understanding human language
  - Related: ["llm", "openai-whisper", "language-understanding"]
  - Chapter: "Chapter 3", Section: "3.4: Speech Recognition and NLP"
- [ ] T076 [P] [US1] Create term file `/frontend/docs/glossary/terms/62-kinematics.md` for "Kinematics"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics"]
  - Definition: ~120 words on motion and geometry of robot movement
  - Related: ["forward-kinematics", "inverse-kinematics", "dynamics"]
  - Chapter: "Chapter 3", Section: "3.1: Kinematics Fundamentals"
- [ ] T077 [P] [US1] Create term file `/frontend/docs/glossary/terms/63-dynamics.md` for "Dynamics"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~120 words on forces and accelerations in robot motion
  - Related: ["kinematics", "control", "physics"]
  - Chapter: "Chapter 3", Section: "3.6: Dynamics and Control"
- [ ] T078 [P] [US1] Create term file `/frontend/docs/glossary/terms/64-manipulation.md` for "Manipulation"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~120 words on robot arm control and object handling
  - Related: ["grasp-planning", "inverse-kinematics", "end-effector"]
  - Chapter: "Chapter 3", Section: "3.3: Manipulation and Grasping"
- [ ] T079 [P] [US1] Create term file `/frontend/docs/glossary/terms/65-gesture-recognition.md` for "Gesture Recognition"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on recognizing human gestures from video/sensors
  - Related: ["computer-vision", "hri", "human-understanding"]
  - Chapter: "Chapter 3", Section: "3.4: Human-Robot Interaction"

### Additional Foundation Terms (~15 terms for cross-domain concepts)

- [ ] T080 [P] [US1] Create term file `/frontend/docs/glossary/terms/66-control-system.md` for "Control System"
  - Acronym: None
  - Categories: ["Control & Learning"]
  - Definition: ~120 words on closed-loop control for robot stability
  - Related: ["feedback", "pid-control", "dynamics"]
  - Chapter: "Chapter 3", Section: "3.6: Control Systems"
- [ ] T081 [P] [US1] Create term file `/frontend/docs/glossary/terms/67-pid-control.md` for "PID Control"
  - Acronym: "Proportional-Integral-Derivative"
  - Categories: ["Control & Learning"]
  - Definition: ~120 words on PID control loops for setpoint regulation
  - Related: ["control-system", "feedback", "stability"]
  - Chapter: "Chapter 3", Section: "3.6: Control Systems"
- [ ] T082 [P] [US1] Create term file `/frontend/docs/glossary/terms/68-computer-vision.md` for "Computer Vision"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~130 words on extracting information from images/video
  - Related: ["image-processing", "object-detection", "semantic-segmentation"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Computer Vision"
- [ ] T083 [P] [US1] Create term file `/frontend/docs/glossary/terms/69-image-processing.md` for "Image Processing"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Software Concepts"]
  - Definition: ~120 words on preprocessing and filtering image data
  - Related: ["computer-vision", "filtering", "feature-detection"]
  - Chapter: "Chapter 2", Section: "2.2: 3D Perception and Computer Vision"
- [ ] T084 [P] [US1] Create term file `/frontend/docs/glossary/terms/70-feedback.md` for "Feedback"
  - Acronym: None
  - Categories: ["Control & Learning"]
  - Definition: ~110 words on feedback mechanism in closed-loop control
  - Related: ["control-system", "sensor", "error-correction"]
  - Chapter: "Chapter 3", Section: "3.6: Control Systems"
- [ ] T085 [P] [US1] Create term file `/frontend/docs/glossary/terms/71-stability.md` for "Stability"
  - Acronym: None
  - Categories: ["Control & Learning", "Kinematics & Dynamics"]
  - Definition: ~120 words on system stability and convergence properties
  - Related: ["control-system", "balance", "zmp"]
  - Chapter: "Chapter 3", Section: "3.6: Control Systems"
- [ ] T086 [P] [US1] Create term file `/frontend/docs/glossary/terms/72-motion-planning.md` for "Motion Planning"
  - Acronym: None
  - Categories: ["Control & Learning", "Kinematics & Dynamics"]
  - Definition: ~120 words on planning collision-free robot trajectories
  - Related: ["path-planning", "trajectory", "obstacle-avoidance"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning and Navigation"
- [ ] T087 [P] [US1] Create term file `/frontend/docs/glossary/terms/73-trajectory.md` for "Trajectory"
  - Acronym: None
  - Categories: ["Kinematics & Dynamics", "Control & Learning"]
  - Definition: ~110 words on robot path over time with velocities
  - Related: ["motion-planning", "path-planning", "dynamics"]
  - Chapter: "Chapter 3", Section: "3.1: Kinematics Fundamentals"
- [ ] T088 [P] [US1] Create term file `/frontend/docs/glossary/terms/74-obstacle-avoidance.md` for "Obstacle Avoidance"
  - Acronym: None
  - Categories: ["Control & Learning", "Perception & Sensing"]
  - Definition: ~120 words on detecting and avoiding obstacles during motion
  - Related: ["path-planning", "costmap", "collision-detection"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning and Navigation"
- [ ] T089 [P] [US1] Create term file `/frontend/docs/glossary/terms/75-collision-detection.md` for "Collision Detection"
  - Acronym: None
  - Categories: ["Perception & Sensing", "Control & Learning"]
  - Definition: ~120 words on detecting when robot collides with environment
  - Related: ["obstacle-avoidance", "physics-engine", "safety"]
  - Chapter: "Chapter 2", Section: "2.3: Path Planning and Navigation"

---

## Phase 4: User Story 2 - Search & Filtering (P2)

**Goal**: Implement category filtering, search ranking, and enhanced glossary UI

**Independent Test**:
- Search box returns matching terms in <3s
- Category filter shows only terms with selected category
- Search results ranked by relevance

- [ ] T090 [US2] Add search UI to `/frontend/docs/glossary/index.md` with:
  - Search input field with placeholder "Search glossary..."
  - Category filter buttons (one per approved category)
  - Display search instructions
- [ ] T091 [US2] Implement category filter in Docusaurus sidebar:
  - Group glossary terms by category in sidebar navigation
  - Enable/disable category visibility via sidebar
- [ ] T092 [US2] Configure Algolia DocSearch in `docusaurus.config.js` for glossary:
  - Crawl only `/docs/glossary/` directory
  - Boost glossary term titles in ranking
  - Index term acronyms separately for search
- [ ] T093 [US2] Create `/frontend/docs/glossary/search-guide.md` explaining:
  - How to search for terms by keyword
  - How to filter by category
  - Keyboard shortcuts for navigation

---

## Phase 5: User Story 3 - Chapter Linking (P3)

**Goal**: Link glossary terms in chapter content, enable bidirectional navigation

**Independent Test**:
- 95% of chapter technical terms have markdown links to glossary
- Clicking term link navigates to glossary entry
- Term pages show "See Also" section with chapter context

- [ ] T094 [US3] Create link pattern guide in `/frontend/docs/glossary/linking-guide.md`:
  - Document markdown link format: `[term name](../../glossary/terms/term-id.md)`
  - Provide link template for common terms
  - List approved term IDs for linking
- [ ] T095 [US3] Review Chapter 1 content `/frontend/docs/chapters/chapter-01/` and link all technical terms:
  - Identify 25+ terms mentioned in Chapter 1
  - Add markdown links to glossary entries
  - Validate links with `npm run glossary:validate-links`
- [ ] T096 [US3] Review Chapter 2 content `/frontend/docs/chapters/chapter-02/` and link all technical terms:
  - Identify 25+ terms mentioned in Chapter 2
  - Add markdown links to glossary entries
  - Validate links with `npm run glossary:validate-links`
- [ ] T097 [US3] Review Chapter 3 content `/frontend/docs/chapters/chapter-03/` and link all technical terms:
  - Identify 25+ terms mentioned in Chapter 3
  - Add markdown links to glossary entries
  - Validate links with `npm run glossary:validate-links`
- [ ] T098 [US3] Add "See Also" section to each glossary term markdown file:
  - List chapters where term appears
  - Link to chapter sections that reference the term
- [ ] T099 [US3] [P] Create build-time report script `/frontend/scripts/report-linking-coverage.js`:
  - Count total technical terms in all chapters
  - Count linked terms vs. unlinked terms
  - Report linking coverage percentage (target: 95%+)
  - Output report to console and `/frontend/docs/glossary/linking-report.md`

---

## Phase 6: Validation & Quality Assurance

**Goal**: Ensure glossary meets all success criteria and constitution requirements

- [ ] T100 Run term count validation: `npm run glossary:validate-terms`
  - Verify 60-80 terms present (SC-001)
  - Output term list to `/frontend/docs/glossary/term-inventory.md`
- [ ] T101 Run category consistency validation: `npm run glossary:validate-categories`
  - Verify all terms have 1-3 approved categories
  - Fix any terms with zero or >3 categories
- [ ] T102 Run link validation: `npm run glossary:validate-links`
  - Verify all cross-references valid (SC-003)
  - Verify no broken links in glossary (test for SC-008)
- [ ] T103 Run glossary build: `npm run build`
  - Verify glossary page loads without errors
  - Check Lighthouse performance (target: <2s page load per SC-005)
- [ ] T104 Test glossary search: Manual test on `/docs/glossary/`
  - Search for 5 random terms
  - Verify results return in <3s (SC-004)
  - Verify results ranked by relevance
- [ ] T105 Test category filtering: Manual test on `/docs/glossary/`
  - Click each category filter
  - Verify filtered results show only terms in selected category
  - Verify count matches filtered category
- [ ] T106 Manual SME review of all 75 terms:
  - Course instructor reviews technical accuracy of definitions (SC-006)
  - Instructor verifies cross-references are appropriate (SC-003)
  - Instructor validates chapter references (SC-005)
  - Document review checklist in `/frontend/docs/glossary/sme-review-checklist.md`
- [ ] T107 Generate glossary report: `/frontend/docs/glossary/glossary-metrics.md` containing:
  - Total term count (target: 60-80)
  - Terms per category breakdown
  - Average cross-references per term (target: 2-5)
  - Linking coverage % (target: 95%+)
  - Build time and page load metrics
  - Validation status for all checks

---

## Phase 7: Deployment & Documentation

**Goal**: Prepare glossary for production and document for future contributors

- [ ] T108 Update main documentation sidebar in `/frontend/docs/sidebar.ts`:
  - Add glossary section to global navigation
  - Set sidebar position (recommend: between Chapters and Reference)
  - Add glossary icon (optional: 📚 or 📖)
- [ ] T109 Update `/frontend/README.md` with glossary contribution instructions:
  - Link to `/frontend/docs/glossary/01-introduction.md`
  - Link to `/frontend/docs/glossary/linking-guide.md`
  - Document validation commands
  - Include example term file structure
- [ ] T110 Create `/frontend/docs/glossary/MAINTENANCE.md`:
  - Update definitions as content evolves
  - Add new terms as chapters expand
  - Remove deprecated terms (with version notes)
  - Instructions for bulk term updates
- [ ] T111 Update project documentation in `/frontend/CONTRIBUTING.md`:
  - Add glossary section with term submission process
  - Link to glossary style guide and metadata schema
  - Document review process for new terms
- [ ] T112 Create glossary performance baseline: `/frontend/docs/glossary/performance-baseline.md`
  - Document Lighthouse score for glossary pages
  - Document search latency benchmark (<3s target)
  - Set up CI checks to prevent regression
- [ ] T113 Add glossary health check to GitHub Actions workflow:
  - Run validation scripts on every PR
  - Fail PR if term count < 60
  - Fail PR if broken links detected
  - Fail PR if category consistency violated
- [ ] T114 Generate final glossary summary: `/frontend/docs/glossary/GLOSSARY-SUMMARY.md`
  - List all 75+ terms with acronyms
  - Show category distribution
  - Show term count per chapter
  - Link to SME review checklist and metrics

---

## Metrics & Success Criteria Mapping

| Success Criteria | Validation Task | Pass Condition |
|------------------|-----------------|----------------|
| SC-001: 60-80 terms | T100 | Term count = 60-80 ✓ |
| SC-002: 95% linking | T099 | Linking coverage ≥ 95% ✓ |
| SC-003: 2-5 cross-refs | T102 | Mean cross-refs = 3.5 ✓ |
| SC-004: <3s search | T104 | Search latency < 3s ✓ |
| SC-005: <2s load | T103 | Lighthouse p95 < 2s ✓ |
| SC-006: 100% accuracy | T106 | SME review complete ✓ |
| SC-007: SSOT | T107 | Single glossary source ✓ |
| SC-008: Accessible | T100 | Available in navigation ✓ |

---

## Task Summary

- **Total Tasks**: 114 (T001-T114)
- **Phase 1 (Setup)**: 8 tasks
- **Phase 2 (Foundational)**: 6 tasks
- **Phase 3 (US1 - Create Terms)**: 75 tasks [PARALLELIZABLE: All T015-T087]
- **Phase 4 (US2 - Search/Filter)**: 4 tasks
- **Phase 5 (US3 - Chapter Linking)**: 6 tasks
- **Phase 6 (Validation)**: 8 tasks
- **Phase 7 (Deployment)**: 7 tasks

**MVP Scope**: Phases 1-3 (Complete glossary with 75 terms) = ~90 tasks
**Full Feature**: All phases 1-7 (with search, filtering, chapter links) = 114 tasks

**Parallel Execution Example (Phase 3)**:
```
Phase 3 can run in parallel for term creation:
- Team Member 1: Create Chapter 1 terms (T015-T035, 21 tasks)
- Team Member 2: Create Chapter 2 terms (T036-T056, 21 tasks)
- Team Member 3: Create Chapter 3 terms (T057-T089, 33 tasks)
- Team Member 4: Create foundation terms (T080-T089, 6 tasks) + coordination

All converge on Phase 6 for validation and SME review.
```

---

**Status**: Ready for Implementation | **Estimated Effort**: 60-80 hours for MVP (Phase 1-3)
