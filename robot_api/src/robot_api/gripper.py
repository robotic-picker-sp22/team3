#! /usr/bin/env python3

# TODO: import ?????????
# TODO: import ???????_msgs.msg
import rospy
import control_msgs.msg
import actionlib

# TODO: ACTION_NAME = ???
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', control_msgs.msg.GripperCommandAction)
        # TODO: Wait for server
        self._client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = self.MIN_EFFORT
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # TODO: Create goal
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = min(self.MAX_EFFORT, max_effort)
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
