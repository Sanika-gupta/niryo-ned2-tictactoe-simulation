#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

class Ned2TicTacToe:
    def __init__(self):
        rospy.init_node("ned2_tictactoe", anonymous=True)

        moveit_commander.roscpp_initialize([])

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)

    def go_to(self, x, y, z, roll=0, pitch=0, yaw=0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Convert RPY → quaternion automatically
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

if __name__ == "__main__":
    robot = Ned2TicTacToe()

    rospy.sleep(2)

    # Example test: Move above the board center
    print("Moving to center...")
    robot.go_to(0.25, 0.0, 0.20)

    print("Done.")

