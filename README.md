# Autonomous Mobile Robot - SLAM Simulation

## Overview

This project investigates Simultaneous Localization and Mapping
(SLAM) techniques for autonomous mobile robots in simulated
environments.

The system uses ROS 2, Gazebo, SLAM Toolbox, RViz2, and Nav2.

The project is being developed as part of an undergraduate
Mechanical Engineering thesis at IFPB.

## Technologies

- Ubuntu Linux
- ROS 2 Lyrical
- Gazebo
- RViz2
- SLAM Toolbox
- Nav2
- Python
- C++

## Project Structure

```text
tcc_slam_ws/
├── src/
│   ├── tcc_slam_description/
│   ├── tcc_slam_bringup/
│   ├── tcc_slam_analysis/
│   └── tcc_slam_ground_truth/
│
├── analisar_experimento.sh
└── README.md
```

## Main Features

- Differential-drive mobile robot simulation
- LiDAR-based SLAM
- Robot odometry and IMU integration
- Autonomous navigation using Nav2
- Ground-truth pose acquisition
- ROS 2 bag data recording
- Python-based experiment analysis

## Evaluation Metrics

The experiment analysis framework includes:

- Robot trajectory evaluation
- Odometry and ground-truth comparison
- Position estimation errors
- Navigation performance analysis

## Development Status

This project is currently under development.

## Author

Erick Cesar

Mechanical Engineering - IFPB
