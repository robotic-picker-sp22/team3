#! /usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy
import copy
import math
from nav_msgs.msg import Odometry
import tf.transformations as tft
import numpy as np
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
PI = math.pi
class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """
    _publisher: rospy.Publisher
    latest_odom: Odometry

    def __init__(self):
        # TODO: Create publisher
        self._publisher = rospy.Publisher(TOPIC, Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        # rospy.init_node(NODE_NAME, anonymous=True)
        # NOTE: not worried abt rate bc this is handled by teleop app
        self.latest_odom = None

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


    def _odom_callback(self, msg):
        self.latest_odom = msg

    def go_forward(self, distance, speed=0.5):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        rate = rospy.Rate(10)
        while not self.latest_odom:
            rate.sleep()
        start = copy.deepcopy(self.latest_odom)
        
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        traveled_distance = abs(math.dist(p2p(self.latest_odom.pose.pose.position), p2p(start.pose.pose.position)))
        while traveled_distance < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            linear_speed = max(0.05, min(speed, abs(distance) - traveled_distance))
            self.move(direction * linear_speed, 0)
            rate.sleep()
            traveled_distance = abs(math.dist(p2p(self.latest_odom.pose.pose.position), p2p(start.pose.pose.position)))


    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        rate = rospy.Rate(10)
        while not self.latest_odom:
            rate.sleep()
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_odom).pose.pose.orientation
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        turn = restrict_turn(angular_distance)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        yaw_start = get_yaw(start)
        curr_turn = angle(yaw_start, get_yaw(self.latest_odom.pose.pose.orientation), turn >= 0)
        prev_turn = curr_turn
        while (turn >= 0 and prev_turn <= curr_turn and turn > curr_turn) \
                or (turn < 0 and prev_turn >= curr_turn and turn < curr_turn):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            angular_speed = max(0.25, min(speed, abs(turn - curr_turn)))
            self.move(0, direction * angular_speed)
            rate.sleep()
            prev_turn = curr_turn
            curr_turn = angle(yaw_start, get_yaw(self.latest_odom.pose.pose.orientation), turn >= 0)

def get_yaw(ori) -> float:
    m = tft.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
    x = m[0,0]
    y = m[1,0]
    theta_rads = math.atan2(y, x)
    return theta_rads

def shouldTurn(z_start: float, z_end: float, z_turn: float) -> bool:
    z_turn = restrict_turn(z_turn)
    curr_turn = angle(z_start, z_end, z_turn >= 0)
    print(f"start: {z_start:0.4f}, end: {z_end: 0.4f}, curr: {curr_turn:0.4f}, expect: {z_turn:0.4f}")
    if z_turn < 0:
        return curr_turn > z_turn
    else:
        return curr_turn < z_turn


def restrict_turn(rad):
    """ Restricts turn between -2PI and 2PI"""
    if rad < 0:
        return -((-rad) % (2*PI))
    else:
        return rad % (2*PI) 

def angle(rad1, rad2, isCCW, tolerance=1E-2):
    """ Gets the angle from rad1 to rad2 going CCW if isCCW is true, else, it goes CW"""

    if abs(rad2 - rad1) < tolerance:
        return 0
    if isCCW:
        if rad2 <= rad1:
            rad2 +=  2 * PI
        return rad2 - rad1
    else:
        if rad2 > rad1:
            rad2 -= 2 * PI
        return rad2 - rad1
    # direction = 1 if isCCW else -1
    # result = ((rad2 + 3*PI) - (rad1 + PI)) * direction % (2*PI) * direction
    # if abs(result % (2*PI) - (2*PI)) < tolerance:
    #     result += direction * tolerance
    #     return result // (2*PI)
    # else:
    #     return result

def p2p(point):
    return [point.x, point.y, point.z]
