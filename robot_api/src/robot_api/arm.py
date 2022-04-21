import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy
# Lab 19
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction                                  

from .arm_joints import ArmJoints

ACTION_NAME = 'arm_controller/follow_joint_trajectory'
TIME_FROM_START = 5


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server()
        # self._move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self._move_group_client = MoveGroupInterface('arm', 'base_link')
        # link to MoveGroupAction doc:
        # http://docs.ros.org/en/indigo/api/moveit_msgs/html/action/MoveGroup.html

    def move_to_pose(self, pose_stamped):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """
        # Fetch manipulation tutorial:
        # http://docs.fetchrobotics.com/manipulation.html#simple-moveit-wave-example
        # Create a goal

        # goal_builder = MoveItGoalBuilder()
        # goal_builder.set_pose_goal(pose_stamped)
        # goal = goal_builder.build()
        # # Send goal and wait forever for result
        # self._move_group_client.send_goal(goal)
        # rospy.loginfo('sent goal to move group action client.')
        # self._move_group_client.wait_for_result()
        # rospy.loginfo('received result from move group action client.')
        # move_group_result = self._move_group_client.get_result()

        self._move_group_client.moveToPose(pose_stamped, 'wrist_roll_link', wait=True)
        move_group_result = self._move_group_client.get_move_action().get_result()

        if move_group_result.error_code.val == MoveItErrorCodes.SUCCESS:
            return None
        else:
            # On failure, return error string
            return moveit_error_string(move_group_result.error_code.val)

    def cancel_all_goals(self):
        '''
        For safety purposes, cancel all the goals sent to the arm
        '''
        self._client.cancel_all_goals() # Your action client from Lab 7
        self._move_group_client.get_move_action().cancel_all_goals() # From this lab

    def move_to_joints(self, arm_joints: ArmJoints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start = rospy.Time(TIME_FROM_START)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = arm_joints.names()
        goal.trajectory.points = [point]
        self._client.send_goal(goal)
        self._client.wait_for_result()
        rospy.loginfo(f'Done moving arm {list(zip(arm_joints.names(), arm_joints.values()))}')



def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'                                      
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'
