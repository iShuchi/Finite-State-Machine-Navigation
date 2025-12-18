# Finite State Machine (FSM) Based Navigation

## Overview

This repository presents the implementation of a custom differentia drive robot capable of performing **Simultaneous Localization and Mapping (SLAM), autonomous navigation** (with dynamic obstacle avoidance), and **mission execution** in a warehouse environment.

![FSM Navigation Demo](videos/demo.gif)

Packages Utilized:

* **2D Mapping** is performed using `slam_toolbox`
* **Localization and Navigation** is performed using `Nav2` stack
* **Mission execution** is performed using a custom **Mission Manager**.

## System Architecture

In the host workspace, I have using following tree architecture:

```text
delhivery_ws/
├── src/
│   └── navigation/
│       ├── diffdrive_bringup/
│       │   ├── .vscode/
│       │   ├── launch/
│       │   │   ├── diffdriveNAV.launch.xml
│       │   │   └── diffdriveSLAM.launch.xml
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       ├── diffdrive_description/
│       │   ├── config/
│       │   │   ├── gazebo_bridge.yaml
│       │   │   ├── NAV2.yaml
│       │   │   └── SLAM.yaml
│       │   │
│       │   ├── maps/
│       │   │   ├── Warehouse.pgm
│       │   │   └── Warehouse.yaml
│       │   │
│       │   ├── rviz/
│       │   │   ├── rviz_config.rviz
│       │   │   └── rviz_config_nav2.rviz
│       │   │
│       │   ├── urdf/
│       │   │   ├── base_mobile.xacro
│       │   │   ├── common_properties.xacro
│       │   │   ├── robot_base.urdf.xacro
│       │   │   └── robot_gazebo.xacro
│       │   │
│       │   ├── world/
│       │   │   └── Warehouse.world
│       │   │
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       ├── mission_manager/
│       │   ├── config/
│       │   │   └── locations.yaml
│       │   │
│       │   ├── include/
│       │   │   └── mission_manager/
│       │   │       └── mission_manager_node.hpp
│       │   │
│       │   ├── src/
│       │   │   └── mission_manager_node.cpp
│       │   │
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│
└── README.md
```

## Execution

Perform git clone to place the given repository inside your workspace,
```bash
cd /workspace/src
git clone https://github.com/iShuchi/Finite-State-Machine-Navigation.git
cd ~/workspace
colcon build
```

Ensure following commands are added to the end of your .bashrc file,
```bash
source /opt/ros/humble/setup.bash
source ~/workspace/install/setup.bash
```

### Launch Mapping

```bash
ros2 launch diffdrive_bringup diffdriveMAP.launch.xml
```

### Keyboard Control (Teleop)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f src/navigation/diffdrive_description/maps/Warehouse
```

This generates an **occupancy grid** with .yaml and .pgm format.

## Simultaneous Localization and Mapping (SLAM)

SLAM-Toolbox uses **graph-based 2D SLAM** approach.

### Mathematical Formulation

Each robot pose is represented as a node in pose graph, with constraints from the optimization problem. Laser scan matching is performed using **correlative scan matching** to minimize drift errors.

## Mapping Additionals

To reduce drift during SLAM, I went through following settings:-

* **Linear velocity** ≤ `0.45 m/s`
* **Angular velocity** ≤ `0.43 rad/s`
* Avoid aggressive rotations
* Move slowly in narrow aisles and corners


## Navigation & Localization (Nav2)

This section deals with use of Navigation Stack (Nav2).

### Launch Navigation

```bash
ros2 launch diffdrive_bringup diffdriveNAV.launch.xml
```

### Map Configuration

If a different map is used, update the following line in:

```
/home/laptop/delhivery_ws/src/navigation/diffdrive_bringup/launch/diffdriveNAV.launch.xml
```

```xml
<arg name="map" value="$(find-pkg-share diffdrive_description)/maps/Warehouse.yaml"/>
```

---

## Localization (AMCL)

Nav2 uses **Adaptive Monte Carlo Localization (AMCL)**.

### Mathematical Model

The robot belief is represented by particles. Particles are resampled based on likelihood computed using the LiDAR likelihood field model. Navigation stack uses **Global** and **Local Path Planners** to dynamically evaluate the path and avoid obstacles during navigation.

## Mission Manager

Pickup, drop, and dock poses are defined by sending Nav2 goals and observing coordinates echoed in the topic:

```bash
ros2 topic echo /goal_pose
```

In the meanwhile, the locations.yaml stores the corresponding calculated yaw and x, y coordinates in location:

```bash
~/delhivery_ws/src/navigation/mission_manager/config/locations.yaml
```

## Mission Execution Flow (FSM)

1. Navigate to **Pickup**
2. Trigger pickup action/alert
3. Navigate to **Drop-off (Assembly Line)**
4. Trigger drop action/alert
5. Navigate to **Dock Station**
6. Wait for next mission (loop)


## Failure Handling (Edge Cases)

* If navigation to any waypoint fails:

  * Failure is reported
  * Mission is aborted
  * Robot **directly navigates to Dock**
* This behavior can be tested by providing unreachable coordinates in `locations.yaml`


## Navigation Additionals

To reduce drift and oscillations:

* Reduce **inflation radius** in `NAV2.yaml` to 0.05
* Reduce **maximum linear velocity (x-axis)** to 0.1

These changes improve path stability and localization accuracy.

## Future Work

* More detailed FSM branches for multiple order permutations
* Sensor fusion using Extended Kalman Filter (EKF)
* Fleet Manager integration via ROS 2 actions