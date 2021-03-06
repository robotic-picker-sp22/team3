#! /usr/bin/env python3

import robot_api
import rospy
from pbd_demo import Command, command_from_string, OPEN, CLOSE, MOVE
from robot_api import Gripper, Arm
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs


class PBDRunner:
    def __init__(self, file_name: str):
        with open(file_name, 'r') as f:
            self._commands = f.readlines()
        self._gripper = Gripper()
        self._arm = Arm()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)


    def run(self):
        for c in self._commands:
            self._run_command(c)


    def _run_command(self, c: str):
        com = command_from_string(c)
        print(f"running command: {c}")

        if com.command == OPEN:
            self._gripper.open()
        elif com.command == CLOSE:
            self._gripper.close()
        elif com.command == MOVE:
            self._arm.move_to_pose_ik(com.pose)
        else:
            print("command not recognized")
    

    def get_transform(self, frame_one, frame_two):
        try:
            trans = self._tf_buffer.lookup_transform(frame_one, frame_two, rospy.Time())
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logerr(f'unable to find transform from {frame_one} to {frame_two}')
            return None


def print_usage():
    print('Usage: rosrun applications pbd_player.py [program_file]')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass




def main():
    rospy.init_node('pbd_player')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    
    runner = PBDRunner(argv[1])
    runner.run()




if __name__ == '__main__':
    main()