#!/usr/bin/env python3

import robot_api
import rospy
# from web_teleop.srv import SetTorso, SetTorsoResponse, SetGripper, SetGripperResponse, SetArm, SetArmResponse, SetHead, SetHeadResponse
from web_teleop.srv import *

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._gripper = robot_api.Gripper()
        self._head = robot_api.Head()
        self._arm = robot_api.Arm()
        self._nav_goal = robot_api.NavGoal()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

    def handle_set_gripper(self, request):
        if request.close > 0:
            self._gripper.close(request.close)
        else:
            self._gripper.open()
        return SetGripperResponse()
    
    def handle_set_arm(self, request):
        # TODO: Set each arm joint
        # joint_list = [request[name[:-6]] for name in robot_api.ArmJoints.names()]
        joint_list = [request.shoulder_pan,
                request.shoulder_lift,
                request.upperarm_roll,
                request.elbow_flex,
                request.forearm_roll,
                request.wrist_flex,
                request.wrist_roll]

        arm_joints = robot_api.ArmJoints.from_list(joint_list)
        self._arm.move_to_joints(arm_joints)
        return SetArmResponse()

    def handle_nav(self, request):
        func_name = request.func_name
        response = NavResponse()
        if func_name == "save_current_pose":
            self._nav_goal.save_current_pose(request.pose_name)
            response.success = True
        elif func_name == "get_locations":
            response.locations = self._nav_goal.get_locations()
            response.success = True
        elif func_name == "goto":
            response.success = self._nav_goal.goto(request.pose_name)
        elif func_name == "delete":
            response.success = self._nav_goal.delete(request.pose_name)
        else:
            response.success = False
        response.locations = self._nav_goal.get_locations()
        return response




def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper, server.handle_set_gripper)
    head_service = rospy.Service("web_teleop/set_head", SetHead, server.handle_set_head)
    arm_service = rospy.Service("web_teleop/set_arm", SetArm, server.handle_set_arm)
    nav_goal = rospy.Service("web_teleop/nav_goal", Nav, server.handle_nav)
    rospy.spin()


if __name__ == '__main__':
    main()