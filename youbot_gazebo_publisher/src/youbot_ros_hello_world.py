#!/usr/bin/env python
# @brief python_file

from __future__ import print_function

import roslib

# roslib.load_manifest('teleop')
import rospy
from numpy import inf, zeros
from geometry_msgs.msg import Twist
import trajectory_msgs.msg as tm

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}


def getKey():
    """
    Function to get keyboard input

    :return: key pressed
    :rtype: char
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    """
    Function to get current velocity (speed and heading direction)
    :param speed: linear velocity (m/sec)
    :param turn: heading direction (radians)

    :return: typeset string useful for displaying current velocity
    :rtype: string
    """
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def createArmPositionCommand(newPositions):
    msg = tm.JointTrajectory()
    point = tm.JointTrajectoryPoint()
    point.positions = newPositions
    point.velocities = zeros(len(newPositions))
    point.accelerations = zeros(len(newPositions))
    point.time_from_start = rospy.Duration(0.5)
    msg.points = [point]
    jointNames = []
    for i in range(5):
        jointNames.append("arm_joint_" + str(i + 1))
    msg.joint_names = jointNames

    msg.header.frame_id = "arm_link_0"
    msg.header.stamp = rospy.Time.now()
    return msg


def createGripperPositionCommand(newPosition):

    msg = tm.JointTrajectory()
    point = tm.JointTrajectoryPoint()

    point.positions = [newPosition, newPosition]
    point.velocities = zeros(2)
    point.accelerations = zeros(2)

    point.time_from_start = rospy.Duration(0.5)
    msg.points = [point]
    msg.joint_names = ["gripper_finger_joint_l", "gripper_finger_joint_r"]

    ## fill message header and sent it out
    msg.header.frame_id = "gripper_finger_joint_l"
    msg.header.stamp = rospy.Time.now()

    return msg


def moveArm():
    armPublisher = rospy.Publisher(
        "/arm_1/arm_controller/command", tm.JointTrajectory, queue_size=1
    )

    jointvalues = [2.95, 1.05, -2.44, 1.73, 2.95]
    msg = createArmPositionCommand(jointvalues)
    armPublisher.publish(msg)
    rospy.sleep(3)
    jointvalues = [0.11, 0.11, -0.11, 0.11, 0.11]

    msg = createArmPositionCommand(jointvalues)
    armPublisher.publish(msg)
    rospy.sleep(3)


def moveGripper():

    gripperPublisher = rospy.Publisher(
        "/arm_1/gripper_controller/command", tm.JointTrajectory, queue_size=1
    )
    msg = createGripperPositionCommand(0.11)
    gripperPublisher.publish(msg)
    rospy.sleep(3)
    msg = createGripperPositionCommand(0)
    gripperPublisher.publish(msg)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("teleop_twist_keyboard")

    # Publisher for velocity command fed to Whiskeye
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    speed = rospy.get_param("~speed", 1.5)  # target linear velocity
    turn = rospy.get_param("~turn", 1.0)  # angle for turning

    # Position variables
    x = 1
    y = 1
    z = 1

    # orientation variables
    th = 1

    twist = Twist()
    twist.linear.x = x * speed
    twist.linear.y = y * speed
    twist.linear.z = z * speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * turn
    pub.publish(twist)
    try:
        print(msg)  # print usage instructions
        print(vels(speed, turn))  # print robot velocity information

        while 1:
            #     key = getKey()  # get the key pressed

            #     if key in moveBindings.keys():
            #         x = moveBindings[key][0]
            #         y = moveBindings[key][1]
            #         z = moveBindings[key][2]
            #         th = moveBindings[key][3]

            #     elif key in speedBindings.keys():
            #         speed = speed * speedBindings[key][0]
            #         turn = turn * speedBindings[key][1]

            #         print(vels(speed, turn))

            #     else:
            #         # Reset parameters if arbitrary key is pressed
            #         x = 0
            #         y = 0
            #         z = 0
            #         th = 0
            #         print(msg)  # Show the usage instructions again

            #         if key == "\x03":  # CTRL-C pressed
            #             break

            #     twist = Twist()
            #     twist.linear.x = x * speed
            #     twist.linear.y = y * speed
            #     twist.linear.z = z * speed
            #     twist.angular.x = 0
            #     twist.angular.y = 0
            #     twist.angular.z = th * turn

            # Publish commands to the robot
            # pub.publish(twist)
            print("moveArm")
            moveArm()
            # print("moveGripper")
            # moveGripper()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
