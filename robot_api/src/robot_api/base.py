#! /usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy

'''
Useful Refrence Documents:

Creating a simple publisher: 
https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

Creating a custom publisher:
http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
'''

# Constants
TOPIC = 'cmd_vel'
NODE_NAME = 'base'

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """
    _publisher: rospy.Publisher

    def __init__(self):
        # TODO: Create publisher
        self._publisher = rospy.Publisher(TOPIC, Twist, queue_size=10)
        # rospy.init_node(NODE_NAME, anonymous=True)
        # NOTE: not worried abt rate bc this is handled by teleop app
        pass

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        msg = Twist()
        # TODO: Fill out msg
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        # TODO: Publish msg
        self._publisher.publish(msg)
        # rospy.loginfo(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0

        # rospy.logerr('Not implemented.')
