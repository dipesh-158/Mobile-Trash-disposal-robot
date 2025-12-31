<p align="center">
  <a href="https://www.youtube.com/watch?v=qpPbaDsw1vQ" target="_blank">
    <img src="https://img.youtube.com/vi/qpPbaDsw1vQ/maxresdefault.jpg"
         alt="Autonomous Trash Collection Robot Demo"
         width="800" />
  </a>
</p>

# Autonomous Trash Collection and Disposal Robot (ROS/Gazebo)

**Author:** Dipesh Budhathoki  
**Environment:** ROS & Gazebo  
**Project Type:** Robotics Simulation | Autonomous Navigation  

---

## Abstract

The concept of smart homes is rapidly becoming a reality, increasing the demand for service robots capable of handling domestic tasks. This project focuses on the design and simulation of an autonomous trash collection and disposal robot within a ROS/Gazebo environment. The robot autonomously navigates a household-like environment, collects trash from a designated area, disposes it in a dumpster, and returns to its starting location as part of a scheduled cycle.

To achieve reliable navigation, obstacle avoidance, and task completion, the robot employs a hybrid control system that integrates sensor data from a laser scanner, IMU, and odometry. A comparison is also demonstrated between robot navigation with and without a path-planning algorithm. This project showcases the application of robotics autonomy principles through simulation.

---

## Introduction

Automation of household tasks improves sanitation, efficiency, and overall quality of living, particularly in waste management. This project presents a robotic system capable of autonomously transporting trash from an indoor area to a disposal location. It contributes to the growing field of home automation and service robotics.

The ROS–Gazebo simulated robot integrates multiple sensors to enable real-time obstacle detection and navigation in a dynamic environment. The system is designed with real-world applicability in mind, emphasizing reliability, adaptability, and autonomous decision-making. The project explores fundamental robotics concepts such as path planning, sensor fusion, and autonomous navigation.

Path planning is a critical challenge in robotics, requiring the robot to determine an optimal route from a start point to a goal while avoiding obstacles. This system addresses the problem by combining reactive navigation strategies with deterministic control elements. Although waypoint-based navigation can improve performance, this project emphasizes real-time sensor dependency by defining only the start and goal points.

---

## Approach

The robot’s control architecture is based on a PID-like hybrid control strategy combined with potential field path planning.

### Hybrid Control System

The navigation system uses attractive and repulsive forces to guide robot motion:

#### Attractive Force
- Guides the robot toward the goal
- Proportional to the distance from the target
- Transitions to a conical potential beyond a defined threshold
- Only the starting point and goal point are defined to encourage real-time sensor-driven navigation

#### Repulsive Force
- Prevents collisions with obstacles
- Proportional to the inverse square of the distance to nearby obstacles
- Activated within a defined sensing range
- Obstacles are detected dynamically using onboard sensors rather than pre-defined maps

---

## Sensors Used

### Laser Sensor (Hokuyo)
- Provides real-time distance measurements
- Used for obstacle detection and repulsive force calculation

### IMU (Inertial Measurement Unit)
- Supplies orientation and angular data
- Helps maintain heading and correct trajectory deviations

### Odometry
- Provides accurate positional feedback
- Ensures reliable navigation toward the goal
- Enables the robot to return to its starting location after task completion

---

## Key Features

- Fully autonomous navigation in a simulated household environment
- Reactive obstacle avoidance using laser sensor data
- Hybrid control combining potential field methods and sensor fusion
- Comparison of navigation performance with and without path planning
- ROS/Gazebo-based simulation suitable for real-world extension

---

## Tools & Technologies

- **ROS (Robot Operating System)**
- **Gazebo Simulation Environment**
- **Laser Scanner (Hokuyo)**
- **IMU & Odometry Sensors**
- **PID-based Control**
- **Potential Field Path Planning**

---

## Future Work

- Integration of object detection for trash identification
- Improved waypoint-based global planning
- Real-world hardware deployment
- Machine learning-based navigation optimization

---

## Acknowledgements

This project was developed as part of an academic robotics and design course, demonstrating practical applications of autonomous navigation and robotic control systems.

