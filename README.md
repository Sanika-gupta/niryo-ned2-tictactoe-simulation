# Niryo Ned2 – Tic Tac Toe Simulation (ROS, MoveIt, Gazebo)

This repository contains a simulation of the Niryo Ned2 robotic arm using
ROS Noetic, MoveIt, and Gazebo. The objective was to model the robot using URDF/Xacro,
visualize it in RViz and Gazebo, and enable basic planning functionality. A simple
tic-tac-toe interaction module was also implemented using Python MoveIt APIs, where
the user inputs a grid position and the system computes the corresponding arm pose.

Due to controller integration limitations, full trajectory execution in Gazebo was
not completed, but the system successfully supports:
- robot modeling
- RViz visualization
- Gazebo spawning
- MoveIt-based motion planning
- tic-tac-toe pose mapping

---


https://github.com/user-attachments/assets/26fb0256-de43-42ed-b7f5-0e34eb6afb60

## Repository Structure
- simulation/
- urdf/ Robot description files
- launch/ Gazebo launch files
- config/ Controller configs
- screenshots/ RViz and Gazebo views

- tictactoe/
- tic_tac_toe.py Terminal-based tic tac toe planner

  <img width="1366" height="768" alt="Screenshot from 2025-11-19 16-51-08" src="https://github.com/user-attachments/assets/dd7f68d2-d884-45d8-94a6-28f6963b6864" />


## How to Run
### 0. Build & Source Workspace
Before running anything:

```bash
cd ~/tic_tac_toe_ws
catkin_make
source devel/setup.bash
```
### 1. Launch Gazebo (Robot + World)
```bash 
roslaunch niryo_robot_gazebo niryo_robot_gazebo_world.launch.xml
```

### 2. Launch MoveIt + RViz
```bash
roslaunch niryo_moveit_config_standalone demo.launch
```
### 3. Run the Tic Tac Toe Controller
```bash
rosrun tic_tac_toe ttt.py
```

## 🔗 Reference to Base Robotics Repository

This simulation builds on the official Niryo Ned ROS repository:

Niryo Robotics (ned_ros)
- https://github.com/NiryoRobotics/ned_ros
