#!/usr/bin/env python

# TODO: import ?????????
import actionlib
# TODO: import ???????_msgs.msg
import control_msgs.msg
# TODO: import trajectory_msgs.msg
import trajectory_msgs.msg
import rospy

# TODO: ACTION_NAME = ???
ACTION_NAME = '/torso_controller/follow_joint_trajectory'
# TODO: JOINT_NAME = ???
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5 # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for server
        self._client.wait_for_server()
        pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        print("Setting height")
        rospy.loginfo("setting height")
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height > self.MAX_HEIGHT:
            height = self.MAX_HEIGHT
        elif height < self.MIN_HEIGHT:
            height = self.MIN_HEIGHT
        else:
            pass
        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions = [height] # Z
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Time(TIME_FROM_START)

        # trajectory = trajectory_msgs.msg.
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names = [JOINT_NAME]
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points = [point]
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
