#!/usr/bin/env python3

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetGripper, SetGripperResponse, SetArm, SetArmResponse, SetHead, SetHeadResponse


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
        return SetArmResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper, server.handle_set_gripper)
    head_service = rospy.Service("web_teleop/set_head", SetHead, server.handle_set_head)
    arm_service = rospy.Service("web_teleop/set_arm", SetArm, server.handle_set_arm)
    rospy.spin()


if __name__ == '__main__':
    main()