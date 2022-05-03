#!/usr/bin/env python3

import rospy
import robot_api
import math

import rospy
import robot_api
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

'''
Uses the location of AR tags as refrences, and can move the robots arm
to reach for one of the AR tags.

BEFORE RUNNING:
- make sure roscore, and gazebo are up
- launch the app which publishes the AR pose markers:
    roslaunch robot_api ar_desktop.launch cam_image_topic:=mock_point_cloud
- run the app to publish the point cloud info so the AR pose marker app
    has a point cloud to work with:
    rosrun applications publish_saved_cloud.py tags.bag
- start Moveit:
    roslaunch robot_api move_group.launch
- move the robot torso to the max height:
    rosrun applications torso_demo.py 0.4
- then you should be able to run this code
'''

TAG_POSE_TOPIC = '/ar_pose_marker'

class ArTagReader(object):

    # List of markers with poses for the AR tags
    markers: list = []  
    _arm: robot_api.Arm

    def __init__(self, arm):
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        self._arm = arm

    def _tag_pose_callback(self, msg):
        ''' Callback for AR Tag pose publisher, msg contains information
        about the location of each AR tag.
        '''
        self.markers = msg.markers

    def wait_for_markers(self):
        ''' Hangs until this receives markers for the AR tags.
        '''
        rospy.loginfo('waiting for markers...')
        while len(self.markers) == 0:
            rospy.sleep(0.1)
        rospy.loginfo('received markers.')

    def move_to_marker(self):
        for marker in self.markers:
            assert(marker.header.frame_id == '/base_link')
            pose: PoseStamped = marker.pose
            pose.pose.position.x -= 0.15
            pose.header.frame_id = '/base_link'
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            error = self._arm.move_to_pose(pose)
            if error is None:
                rospy.loginfo('Moved to marker {}'.format(marker.id))
                return
            else:
                rospy.logwarn('Failed to move to marker {}'.format(marker.id))
        rospy.logerr('Failed to move to any markers!')


def main():
    rospy.init_node('hallunicated_reach')
    from arm_demo import wait_for_time
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5 
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75

    arm = robot_api.Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    arm.move_to_pose(start)

    reader = ArTagReader(arm)
    reader.wait_for_markers()
    reader.move_to_marker()


if __name__ == "__main__":
    main()