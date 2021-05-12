#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64 
import trajectory_msgs.msg as tm
from numpy import inf, zeros, ones
from geometry_msgs.msg import Twist

def moveArm():
    armPublisher = rospy.Publisher(
        "/arm_1/arm_controller/command", tm.JointTrajectory, queue_size=1
    )

    jointvalues = [2.95, 1.05, -2.44, 1.73, 2.95]
    msg = createArmPositionCommand(jointvalues)
    armPublisher.publish(msg)
    print("Para arriba")
    rospy.sleep(3)
    jointvalues = [0.11, 0.11, -0.11, 0.11, 0.11]

    msg = createArmPositionCommand(jointvalues)
    armPublisher.publish(msg)
    print("Para abajo")
    rospy.sleep(3)


def createArmPositionCommand(newPositions):
    msg = tm.JointTrajectory()
    point = tm.JointTrajectoryPoint()
    point.positions = newPositions
    vel = 30.0
    point.velocities =  ones(len(newPositions))*vel
    point.accelerations = ones(len(newPositions))*vel
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
    point.velocities = ones(2)
    point.accelerations = zeros(2)
    point.time_from_start = rospy.Duration(0.5)
    msg.points = [point]
    msg.joint_names = ["gripper_finger_joint_l", "gripper_finger_joint_r"]
    msg.header.frame_id = "gripper_finger_joint_l"
    msg.header.stamp = rospy.Time.now()

    return msg
    
def moveGripper():

    gripperPublisher = rospy.Publisher(
        "/arm_1/gripper_controller/command", tm.JointTrajectory, queue_size=1
    )
    msg = createGripperPositionCommand(0.11)
    gripperPublisher.publish(msg)
    print("Open")
    rospy.sleep(3)
    msg = createGripperPositionCommand(0.0)
    gripperPublisher.publish(msg)
    print("Close")
    rospy.sleep(3)

def gripper():


	rospy.init_node('youbot_teleop')


	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	twist = Twist()
	vel = 1.5
	twist.linear.x = vel
	twist.linear.y = vel
	twist.linear.z = vel
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = vel

	# Publish commands to the robot
	pub.publish(twist)
    	moveGripper()
	rospy.spin()
	

        
if __name__ == '__main__':
    
    gripper()

