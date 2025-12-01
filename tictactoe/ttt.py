#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

##
# ROS + MoveIt Initialization

moveit_commander.roscpp_initialize([])
rospy.init_node("tic_tac_toe_controller")

# MoveIt group for NED2 arm
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_planning_time(10)
arm.set_num_planning_attempts(20)
arm.set_max_velocity_scaling_factor(0.3)

# Force planner to use cartesian-friendly behavior
arm.allow_replanning(True)

##
# Board Coordinates

# Safe height above board
Z_HEIGHT = 0.18

board_positions = {
    1: [-0.10, 0.15, Z_HEIGHT],
    2: [-0.10, 0.25, Z_HEIGHT],
    3: [-0.10, 0.35, Z_HEIGHT],

    4: [ 0.00, 0.15, Z_HEIGHT],
    5: [ 0.00, 0.25, Z_HEIGHT],
    6: [ 0.00, 0.35, Z_HEIGHT],

    7: [ 0.10, 0.15, Z_HEIGHT],
    8: [ 0.10, 0.25, Z_HEIGHT],
    9: [ 0.10, 0.35, Z_HEIGHT],
}

##
# Helper Functions


def move_to_cell(cell):
    """Moves the robot above a selected tic-tac-toe cell safely."""
    x, y, z = board_positions[cell]

    # Step 1 — move HIGH above the target (safe)
    pose = arm.get_current_pose().pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z + 0.10  # Approach from above

    # Orientation: end effector pointing straight down
    qx, qy, qz, qw = quaternion_from_euler(3.14, 0.0, 1.57)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    arm.set_pose_target(pose)
    arm.go(wait=True)
    arm.clear_pose_targets()

    # Step 2 — lower to target Z height
    pose.position.z = z
    arm.set_pose_target(pose)
    arm.go(wait=True)
    arm.clear_pose_targets()


def go_home():
    """Move robot to known safe home pose."""
    try:
        arm.set_named_target("home")
        arm.go(wait=True)
        arm.stop()
    except:
        pass

##
# Start in HOME pose
go_home()

##
# Tic-Tac-Toe Logic


board = [" "] * 9
turn = "X"

def print_board():
    print("\n")
    print(board[0], "|", board[1], "|", board[2])
    print("--+---+--")
    print(board[3], "|", board[4], "|", board[5])
    print("--+---+--")
    print(board[6], "|", board[7], "|", board[8])
    print("\n")

##
# Main Game Loop

while not rospy.is_shutdown():

    print_board()
    print("Turn:", turn)

    try:
        cell = int(input("Enter cell (1–9): "))
    except:
        print("Invalid input.")
        continue

    if cell < 1 or cell > 9:
        print("Choose a number between 1–9.")
        continue

    if board[cell - 1] != " ":
        print("Cell already taken!")
        continue

    # Command robot to move
    print("[INFO] Moving robot to cell", cell)
    move_to_cell(cell)

    # Update board
    board[cell - 1] = turn

    # Next turn
    turn = "O" if turn == "X" else "X"
