# youbot_gazebo_publisher
Sample program for simulation of KUKA youBot in Gazebo simulator using ROS.

As you may notice this repository is a fork of another one. My intention was to rewrite the robot control program in the simulator from the C++ language to Python.

### What's new?
* youbot_ros_hello_world.py: It is almost a copy of the C++ file of the same name. The functions have exactly the same name and the same utility.
* gripper.py: File dedicated only to move the robot manipulator, this file has however functions to move the whole arm.
* listener.py: File that creates a node that listens to the odometry topics and states of the robot manipulator/gripper controller. 

### Technologies
* [YouBot Gazebo] 
* [Python] - 2.7.16
* [RosPy]


### Tips and tricks
* As you probably already know, to list all the topics that are active you can use the command: rostopic list
* Topics have different message types associated with them, these can be verified using: rostopic info <topic_name>
* To know the information to publish in each type of message you can use:   rosmsg show <msg_type>

License
----

MIT



   [YouBot Gazebo]: <http://www.youbot-store.com/developers/ros-gazebo-simulation>
   [Python]: <https://www.python.org/downloads/release/python-2716/>
   [RosPy]: <http://wiki.ros.org/rospy>

