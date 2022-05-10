#!/usr/bin/env python3

import rospy
import robot_api
import math

import rospy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

TAG_POSE_TOPIC = '/ar_pose_marker'

class BinMarkerCropper(object):

    markers: list = []

    def __init__(self):
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        rospy.loginfo('bin marker cropper initialized.')

    

    def _tag_pose_callback(self, msg):
        ''' Callback for AR Tag pose publisher, msg contains information
        about the location of each AR tag.
        '''
        self.markers = msg.markers
        # rospy.loginfo(f'found {len(self.markers)} markers.')
        self.crop_for_marker()


    # def wait_for_markers(self):
    #     ''' Hangs until this receives markers for the AR tags.
    #     '''
    #     rospy.loginfo('waiting for markers...')
    #     while len(self.markers) == 0:
    #         rospy.sleep(0.1)
    #     rospy.loginfo('received markers.')


    def _crop(self, x, y, z):
        rospy.loginfo(f'cropping to ({x}, {y}, {z})')
        DEFAULT_WID = 0.4
        DEFAULT_HEI = 0.1
        DEFAULT_DEP = 0.1
        min_x = x - DEFAULT_WID/2
        min_y = y - DEFAULT_HEI/2
        min_z = z - DEFAULT_DEP/2
        max_x = x + DEFAULT_WID/2 
        max_y = y + DEFAULT_HEI/2 
        max_z = z + DEFAULT_DEP/2 
        rospy.set_param("crop_min_x", min_x)    
        rospy.set_param("crop_min_y", min_y)
        rospy.set_param("crop_min_z", min_z)
        rospy.set_param("crop_max_x", max_x)
        rospy.set_param("crop_max_y", max_y)
        rospy.set_param("crop_max_z", max_z)



    def crop_for_marker(self):
        if len(self.markers) < 1:
            rospy.logerr('Could not find any markers!')
        for marker in self.markers:
            assert(marker.header.frame_id == '/base_link')
            p = marker.pose.pose.position
            self._crop(p.x, p.y, p.z)
            return
        rospy.logerr('Failed to find any markers!')


def main():
    rospy.init_node('bin_marker_cropper')
    from arm_demo import wait_for_time
    wait_for_time()

    cropper = BinMarkerCropper()
    rospy.loginfo('spinning...')
    rospy.spin()


if __name__ == "__main__":
    main()