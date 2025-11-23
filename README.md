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

## Repository Structure

- simulation/
- urdf/ Robot description files
- launch/ Gazebo launch files
- config/ Controller configs
- screenshots/ RViz and Gazebo views

- tictactoe/
- tic_tac_toe.py Terminal-based tic tac toe planner
