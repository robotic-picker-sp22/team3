import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy
# Lab 19
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
# from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction, MoveGroupGoal, OrientationConstraint
from .arm_joints import ArmJoints
from robot_controllers_msgs.msg import QueryControllerStatesAction

# Joint State
from sensor_msgs.msg import JointState

ACTION_NAME = 'arm_controller/follow_joint_trajectory'
# rf for relax / freeze
RF_ACTION_NAME = '/query_controller_states'
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
        print('follow joint trajectory initialized')
        self._rf_client = actionlib.SimpleActionClient(RF_ACTION_NAME, QueryControllerStatesAction)
        self._rf_client.wait_for_server()
        print('query controller states initialized')
        self._move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self._move_group_client.wait_for_server()
        print('move group client initialized')
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._joint_state = None
        self._joint_state_subscriber = rospy.Subscriber('joint_states', JointState, self._set_joint_state)
        # link to MoveGroupAction doc:
        # http://docs.ros.org/en/indigo/api/moveit_msgs/html/action/MoveGroup.html
        print('arm object initialized')


    def _set_joint_state(self, msg: JointState):
        self._joint_state = msg
    
    def _apply_default_constraints(self, goal_builder: MoveItGoalBuilder):
        ''' 
        Get link state:
        rosservice call /gazebo/get_link_state 'link_name: fetch::elbow_flex_link'
        '''
        # Restrict shoulder height
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'shoulder_lift_link'
        oc.orientation.x = 0
        oc.orientation.y = 0.5866522458554224 #1.2  # pointing downward
        oc.orientation.z = 0    
        oc.orientation.w = 1
        oc.absolute_x_axis_tolerance = 3.14
        oc.absolute_y_axis_tolerance = 0.4 # raise up to about flat
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0
        # goal_builder.add_path_orientation_constraint(oc)

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'gripper_link'
        oc.absolute_x_axis_tolerance = 0.2 
        oc.absolute_y_axis_tolerance = 0.2 # raise up to about flat
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0
        # goal_builder.add_path_orientation_constraint(oc)
       
        # links = ArmJoints.names()
        # for link in links:
        #     oc = OrientationConstraint()
        #     oc.header.frame_id = link
        #     oc.link_name = link
        #     oc.absolute_x_axis_tolerance = 3.14
        #     oc.absolute_y_axis_tolerance = 3.14
        #     oc.absolute_z_axis_tolerance = 3.14
        #     oc.weight = 0.8
        #     goal_builder.add_path_orientation_constraint(oc)
        
        # oc = OrientationConstraint()
        # oc.header.frame_id = 'base_link'
        # oc.link_name = 'elbow_flex_link'
        # oc.orientation.x = 0
        # oc.orientation.y = -0.9967683264558241
        # oc.orientation.z = 0    
        # oc.orientation.w = 1
        # oc.absolute_x_axis_tolerance = 3.14
        # oc.absolute_y_axis_tolerance = 0.2
        # oc.absolute_z_axis_tolerance = 3.14
        # oc.weight = 0.8
        # goal_builder.add_path_orientation_constraint(oc)

    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint: OrientationConstraint=None):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.
            orientation_constraint: the orientation the object should be at all times
        Returns:
            string describing the error if an error occurred, else None.
        """
        # Fetch manipulation tutorial:
        # http://docs.fetchrobotics.com/manipulation.html#simple-moveit-wave-example
        
        # Create a goal
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        if self._joint_state is not None:
            goal_builder.start_state.joint_state = self._joint_state
        # self._apply_default_constraints(goal_builder)
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)
        goal: MoveGroupGoal = goal_builder.build()

        # Send goal and await result
        self._move_group_client.send_goal(goal)
        self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        move_group_result = self._move_group_client.get_result()
        if move_group_result != None and move_group_result.error_code.val == MoveItErrorCodes.SUCCESS:
            return None
        else:
            # On failure, return error string
            if move_group_result == None:
                return moveit_error_string(MoveItErrorCodes.TIMED_OUT)
            else:
                return moveit_error_string(move_group_result.error_code.val)

    def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    def cancel_all_goals(self):
        '''
        For safety purposes, cancel all the goals sent to the arm
        '''
        self._client.cancel_all_goals() # Your action client from Lab 7
        self._move_group_client.cancel_all_goals() # From this lab

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

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5), print=True):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        request.ik_request.robot_state.joint_state = self._joint_state
        response = self._compute_ik(request)
        error_str = moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state: JointState = response.solution.joint_state
        if print:
            for name, position in zip(joint_state.name, joint_state.position):
                if name in ArmJoints.names():
                    rospy.loginfo('{}: {}'.format(name, position))
        return True


    """Relaxes the arm so a human can move it
    Only has an effect if the robot is not in simulation, unless force is true

    Args:
        force: forces motion control to be stopped even if robot is in simulation
    """
    def stop_motion_control(self, force=False):
        if force or not rospy.get_param("/use_sim_time", False):
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            self._rf_client.send_goal(goal)
            self._rf_client.wait_for_result()

    """Starts motion control to allow robot movement
    If stop_motion_control is called, this must be called before any moveit commands are run
    """
    def start_motion_control(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._rf_client.send_goal(goal)
        self._rf_client.wait_for_result()


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
