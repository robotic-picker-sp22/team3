#! /usr/bin/env python3

import rospy
import robot_api
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()
    tfBuffer: Buffer = Buffer()
    listener = TransformListener(tfBuffer)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            # The two frames might need to be switched
            trans = tfBuffer.lookup_transform('base_link','gripper_link', rospy.Time())
            rospy.loginfo(trans)
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()

if __name__ == '__main__':
    main()