#!/usr/bin/env python3

'''
NOTE: existing head.py file was completely different from code provided
for lab 6, so I moved it to the file old_head.py --Noah

Refrence pages:
joint names: http://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques
topics and types: http://docs.fetchrobotics.com/api_overview.html 
FollowJointTrajectoryAction: http://docs.ros.org/en/electric/api/control_msgs/html/action/FollowJointTrajectory.html
PointHeadAction: http://docs.ros.org/en/electric/api/control_msgs/html/action/PointHeadAction.html

'''

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import math
import rospy

LOOK_AT_ACTION_NAME = '/head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory' # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = math.radians(-90)  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.radians(90)  # TODO: Maximum pan angle, in radians.
    MIN_TILT = math.radians(-45)  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.radians(90)  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self._lookat_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction) 
        self._pantilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for both servers
        self._lookat_client.wait_for_server()
        self._pantilt_client.wait_for_server()
        rospy.loginfo("Head initialized.")

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        goal = control_msgs.msg.PointHeadGoal()
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        point = goal.target.point
        point.x, point.y, point.z = x, y, z
        goal.target.header.frame_id = frame_id
        goal.min_duration = rospy.Time(1)
        # TODO: Send the goal
        self._lookat_client.send_goal(goal)
        # TODO: Wait for result
        self._lookat_client.wait_for_result()
        rospy.loginfo(f'Done sending goal for {frame_id} ({x}, {y}, {z}).')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        pan = max(self.MIN_PAN, pan)
        pan = min(self.MAX_PAN, pan)
        tilt = max(self.MIN_TILT, tilt)
        tilt = min(self.MAX_TILT, tilt)
        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set positions of the two joints in the trajectory point
        point.positions = [pan, tilt]
        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Time(PAN_TILT_TIME)
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint names to the list
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        # TODO: Add trajectory point created above to trajectory
        goal.trajectory.points = [point]

        # TODO: Send the goal
        self._pantilt_client.send_goal(goal)
        # TODO: Wait for result
        self._pantilt_client.wait_for_result()
        rospy.loginfo(f'Done pan tilt goal pan: {pan} tilt: {tilt}')