#!/usr/bin/env python3

import rospy
import robot_api
from geometry_msgs.msg import PoseStamped
from robot_api import Gripper, Arm
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

EE_LINK = 'gripper_link'

class CommandLineApp(object):

    def __init__(self):
        self._gripper = Gripper()
        self._arm = Arm()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)

        self._commands = []

        #self._joint_state_subscriber = rospy.Subscriber('joint_states', JointState, self._set_joint_state)
        print('command line app starting')

    def spin(self):
        while True:
            command_text = input("Next Command (open, close, move, save): ").strip().lower()
            command = Command(command_text)
            if command_text == OPEN:
                self._gripper.open()
            elif command_text == CLOSE:
                self._gripper.close()
            elif command_text == MOVE:
                frame = input("Frame (base_link, odom, map): ").strip()
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = frame
                # get transform from frame to gripper_link
                gripper_pos = self.get_transform(frame, EE_LINK)
                if gripper_pos is None:
                    print('invalid reference frame, unable to register command')
                    continue
                pose_stamped.pose.position.x = gripper_pos.transform.translation.x
                pose_stamped.pose.position.y = gripper_pos.transform.translation.y
                pose_stamped.pose.position.z = gripper_pos.transform.translation.z
                pose_stamped.pose.orientation = gripper_pos.transform.rotation
                command.frame = frame
                command.pose = pose_stamped
            elif command_text == SAVE:
                file_name = input("Enter file to save program: ").strip()
                f = open(file_name, 'w')
                for c in self._commands:
                    f.write(command_to_string(c))
                    f.write('\n')
                f.close()
                print("saved program, shutting down...")
                break
            else:
                print("Command not recognized.")
                continue
            self._commands.append(command)
    
    def get_transform(self, frame_one, frame_two):
        try:
            trans = self._tf_buffer.lookup_transform(frame_one, frame_two, rospy.Time())
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logerr(f'unable to find transform from {frame_one} to {frame_two}')
            return None



OPEN = 'open'
CLOSE = 'close'
MOVE = 'move'
SAVE = 'save'

class Command:
    def __init__(self, command:str, pose:PoseStamped = None):
        self.command = command
        self.pose = pose

def command_to_string(c: Command):
    if c.command != MOVE:
        return c.command
    return ' '.join([str(x) for x in [c.command, 
            c.pose.header.frame_id, 
            c.pose.pose.position.x, 
            c.pose.pose.position.y, 
            c.pose.pose.position.z,
            c.pose.pose.orientation.x,
            c.pose.pose.orientation.y,
            c.pose.pose.orientation.z,
            c.pose.pose.orientation.w]])

def command_from_string(s: str) -> Command:
    data = s.split()
    c = Command(data[0])
    if data[0] != MOVE:
        return c
    c.pose = PoseStamped()
    c.pose.header.frame_id = data[1]
    c.pose.pose.position.x = float(data[2])
    c.pose.pose.position.y = float(data[3])
    c.pose.pose.position.z = float(data[4])
    c.pose.pose.orientation.x = float(data[5])
    c.pose.pose.orientation.y = float(data[6])
    c.pose.pose.orientation.z = float(data[7])
    c.pose.pose.orientation.w = float(data[8])
    return c

def main():
    rospy.init_node('pbd_demo')
    from arm_demo import wait_for_time
    wait_for_time()

    arm = robot_api.Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    cla = CommandLineApp()
    cla.spin()


if __name__ == "__main__":
    main()