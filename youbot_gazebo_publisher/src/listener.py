#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState


def callback_odom(data):
    print("odom\n" + str(data))


def callback_JointTrajectory(data):
    print("gripper_controller/command\n" + str(data))


def callback_gripper_JointTrajectory(data):
    print("gripper_controller\n" + str(data))


def listener():
    rospy.init_node("listener12", anonymous=True)

    # rospy.Subscriber("/odom", Odometry, callback_odom)
    # rospy.Subscriber("/arm_1/gripper_controller/command", JointTrajectory, callback_JointTrajectory)
    rospy.Subscriber(
        "/arm_1/gripper_controller/state",
        JointTrajectoryControllerState,
        callback_gripper_JointTrajectory,
    )
    rospy.spin()


if __name__ == "__main__":
    listener()
