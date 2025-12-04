---
id: lesson-03-perception
title: "Lesson 3: Visual Perception and SLAM"
---

## 2.3: Visual Perception Foundations

For robots to navigate and manipulate, they must perceive their environment through **visual perception**, which transforms raw sensor data (pixels, point clouds) into actionable information.

### 2.3.1: The Pinhole Camera Model

The **pinhole camera model** projects 3D world points onto a 2D image plane through geometric projection:

$$s\begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{K} [\mathbf{R} | \mathbf{t}] \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}$$

The camera intrinsic matrix K encodes focal lengths and principal point. Real lenses introduce distortion modeled by radial and tangential parameters.

### 2.3.2: RGB-D and LiDAR Sensing

**RGB-D Sensors** combine color images with depth maps:
- Time-of-flight: Measure light travel time
- Structured light: Project known patterns and triangulate
- Stereo: Triangulate from two camera views

**Point Cloud Processing** converts depth images to 3D representations for scene understanding.

**LiDAR Sensors** emit laser pulses to build dense 3D maps with high accuracy and range.

### 2.3.3: Feature Detection and Matching

Perception systems extract **visual features** for tracking and object detection:
- **ORB**: Fast, rotation-invariant, used in SLAM
- **SIFT**: Highly distinctive but computationally expensive
- **AKAZE**: Fast with binary descriptors
- **Harris**: Simple corner detection

---

## 2.4: SLAM (Simultaneous Localization and Mapping)

**SLAM** is the fundamental robotics problem: localize in an unknown environment while building a map.

### 2.4.1: Problem Formulation

SLAM estimates both robot pose $\mathbf{x}_k$ (position and rotation) and map $\mathbf{m}$ from:
- Control inputs (odometry, IMU)
- Observations (camera images, depth)

### 2.4.2: Visual SLAM (V-SLAM)

**ORB-SLAM2** is a popular visual SLAM system with three phases:
1. **Tracking**: Feature tracking between frames
2. **Local Mapping**: Insert features into map, optimize recent poses (bundle adjustment)
3. **Loop Closure**: Detect revisited locations and correct accumulated drift

**Bundle Adjustment** optimizes camera poses and 3D points by minimizing reprojection error.

### 2.4.3: Loop Closure Detection

As odometry errors accumulate, **loop closure** recognizes when the robot returns to a known location and corrects drift through **pose graph optimization**.

### 2.4.4: Visual-Inertial Odometry (VIO)

VIO fuses visual features with IMU measurements for:
- Fast motion response
- Robustness in textureless scenes
- Full 3-DOF motion estimation
