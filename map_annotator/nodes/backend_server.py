#!/usr/bin/env python3

from colorama import Back
import rospy
from map_annotator import NavGoal
from map_annotator.msg import UserAction, PoseNames

'''
Commands for checking the backend
echo the topic that gets published: rostopic echo map_annotator/pose_names
publish a user action from the command line: rostopic pub -l  -1 /map_annotator/user_actions map_annotator/UserAction '{command: create, name: markername}'
start the backend server: roslaunch map_annotator map_annotator.launch
also start the backend server: rosrun map_annotator backend_server.py

'''

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


SUB_TOPIC = '/map_annotator/user_actions'
PUB_TOPIC = '/map_annotator/pose_names'

class BackendServer(object):
    def __init__(self):
        '''
        Creates the NavGoal object, the subscriber for getting frontend user input,
        and a publisher to advertise the pose names
        '''
        self._nav = NavGoal()
        self._sub = rospy.Subscriber(SUB_TOPIC, UserAction, self._action_callback)
        self._pub = rospy.Publisher(PUB_TOPIC, PoseNames, queue_size=10, latch=True)
        print("Map annotator backend server up!")

    def _action_callback(self, msg):
        '''
        Called when a new UserAction is published, reads the command and name from the 
        message, then calls the correct NavGoal function
        '''
        command = msg.command
        if command == UserAction.CREATE:
            self._nav.create_new_pose(msg.name)
            self._pub.publish(self._nav.get_locations())
        elif command == UserAction.DELETE:
            self._nav.delete(msg.name)
            self._pub.publish(self._nav.get_locations())
        elif command == UserAction.GOTO:
            self._nav.goto(msg.name)
        else:
            rospy.logerr(f'could not interpret command {command}')

def main():
    rospy.init_node('map_annotator_backend')
    wait_for_time()
    server = BackendServer()
    rospy.spin()

if __name__ == '__main__':
    main()