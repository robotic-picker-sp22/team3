#!/usr/bin/env python3

from tkinter import Y
import rospy
import robot_api
import math

import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf.transformations as tft
from visualization_msgs.msg import Marker

'''
Only works if cropper is already running:
rosrun perception point_cloud_demo labels/

For real robot:
rosrun perception point_cloud_demo labels/ cloud_in:=/head_camera/depth_registered/points
'''

TAG_POSE_TOPIC = '/ar_pose_marker'

"""
Shelf map
+--------+----+----+----+----+
|        | 10 | 20 | 30 | 40 |
|   00   |    |    |    |    |
|        +----+----+----+----+
+--------+ 07 | 17 | 27 | 37 |
|        |    |    |    |    |
|   01   +----+----+----+----+
|        | 08 | 18 | 28 | 38 |
+--------+    |    |    |    |
|        +----+----+----+----+
|   02   | 09 | 19 | 29 | 39 |
|        |    |    |    |    |
+--------+----+----+----+----+
|        | 10 | 20 | 30 | 40 |
|   03   |    |    |    |    |
|        +----+----+----+----+
+--------+ 11 | 21 | 31 | 41 |
|        |    |    |    |    |
|   04   +----+----+----+----+
|        | 12 | 22 | 32 | 42 |
+--------+    |    |    |    |
|        +----+----+----+----+
|   05   | 13 | 23 | 33 | 43 |
|        |    |    |    |    |
+--------+----+----+----+----+
         | 14 | 24 | 34 | 44 |
         |    |    |    |    |
         +----+----+----+----+
         | 15 | 25 | 35 | 45 |
         |    |    |    |    |
         +----+----+----+----+

"""

# relative to the shelf top left front
crop_bounds: list = [
#   [min_x, max_x, min_y, max_y, min_z, max_z], # shelf
    [ 0.00,  0.31,  0.00,  0.22,  0.20, -0.28], # 00
    [ 0.00,  0.31,  0.23,  0.04,  0.20, -0.28], # 01*
    [ 0.00,  0.31, -0.18,  0.04,  0.20, -0.28], # 02
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 03
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 04
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 05
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 06
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 07
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 08
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 09
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 10
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 11
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 12
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 13
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 14
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 15
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 16
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 17
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 18
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 19
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 20
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 21
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 22
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 23
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 24
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 25
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 26
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 27
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 28
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 29
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 30
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 31
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 32
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 33
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 34
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 35
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 36
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 37
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 38
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 39
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 40
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 41
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 42
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 43
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 44
    [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28]  # 45
]                                              

class BinMarkerCropper(object):

    markers: list = []

    def __init__(self):
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        rospy.loginfo('bin marker cropper initialized.')
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    

    def _tag_pose_callback(self, msg):
        ''' Callback for AR Tag pose publisher, msg contains information
        about the location of each AR tag.
        '''
        self.markers = msg.markers
        # rospy.loginfo(f'found {len(self.markers)} markers.')
        # shelf: int = int(rospy.get_param('crop_shelf', 1))
        # self.crop_shelf_for_marker(shelf)

        # Crop to the shelf below the one with the marker
        self._crop(-0.237, -0.115, -0.1)


    # def wait_for_markers(self):
    #     ''' Hangs until this receives markers for the AR tags.
    #     '''
    #     rospy.loginfo('waiting for markers...')
    #     while len(self.markers) == 0:
    #         rospy.sleep(0.1)
    #     rospy.loginfo('received markers.')


    def _crop(self, x, y, z):
        rospy.loginfo(f'cropping to ({x}, {y}, {z})')
        DEFAULT_WID = 0.24
        DEFAULT_HEI = 0.14
        DEFAULT_DEP = 0.24
        min_x = x - DEFAULT_HEI/2
        min_y = y - DEFAULT_WID/2
        min_z = z - DEFAULT_DEP/2
        max_x = x + DEFAULT_HEI/2 
        max_y = y + DEFAULT_WID/2 
        max_z = z + DEFAULT_DEP/2 
        rospy.set_param("crop_min_x", min_x)    
        rospy.set_param("crop_min_y", min_y)
        rospy.set_param("crop_min_z", min_z)
        rospy.set_param("crop_max_x", max_x)
        rospy.set_param("crop_max_y", max_y)
        rospy.set_param("crop_max_z", max_z)
        self._pub_marker(min_x, max_x, min_y, max_y, min_z, max_z)


    def _crop_dims(self, x, y, z, w, h, d):
        rospy.loginfo(f'cropping to ({x}, {y}, {z})')
        y += -0.14
        z += -0.08
        DEFAULT_WID = w
        DEFAULT_HEI = h
        DEFAULT_DEP = d
        min_x = x - DEFAULT_DEP/2
        min_y = y - DEFAULT_WID/2
        min_z = z - DEFAULT_HEI/2
        max_x = x + DEFAULT_DEP/2 
        max_y = y + DEFAULT_WID/2 
        max_z = z + DEFAULT_HEI/2 
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

    
    def crop_shelf_for_marker(self, shelf: int):
        '''
        Args:
            shelf - 
        '''
        bounds = self._get_shelf_bounds(shelf)
        x = bounds[0]
        w = bounds[1]
        y = bounds[2]
        h = bounds[3]
        z = bounds[4]
        d = bounds[5]
        rospy.loginfo(f'cropping to ({x}, {y}, {z}), ({w}, {h}, {d})')
        x += 0.03
        y += 0.03
        max_x = x - w
        max_y = y - h
        max_z = z + d
        rospy.set_param("crop_min_x", max_x)    
        rospy.set_param("crop_min_y", max_y)
        rospy.set_param("crop_min_z", z)
        rospy.set_param("crop_max_x", x)
        rospy.set_param("crop_max_y", y)
        rospy.set_param("crop_max_z", max_z)
        self._pub_marker(x, w, y, h, z, d)

    def _get_shelf_bounds(self, shelf: int):
        '''
        Returns the bounding box for a given shelf
        Args:
            shelf: the index of the shelf to get the bounds for 
        Returns: the min and max x, y, z values for the shelf (6 total values)
        '''
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0
        min_z = -0.28
        max_z = 0.5
        padding = 0.01
        if (shelf < 10):
            min_x = 0
            max_x = 0.31
            min_y = shelf * 0.22
            max_y = 0.22
        else:
            row = shelf % 10
            col = shelf // 10
            min_x = 0.31 + col * 0.15
            max_x = 0.15
            min_y = row * 0.13
            max_y = 0.13

        max_x -= padding
        max_y -= padding
        return min_x, max_x, min_y, max_y, min_z, max_z

    def _pub_marker(self, min_x, max_x, min_y, max_y, min_z, max_z):
        marker = Marker(type=Marker.CUBE)
        marker.header.frame_id = "ar_marker_15"
        # Pose is at the center of the marker
        marker.pose = Pose()
        marker.pose.position.x = (max_x + min_x) / 2
        marker.pose.position.y = (max_y + min_y) / 2
        marker.pose.position.z = (max_z + min_z) / 2
        marker.scale = Vector3()
        marker.scale.x = max_x - min_x
        marker.scale.y = max_y - min_y
        marker.scale.z = max_z - min_z
        marker.color.r = 1
        marker.color.a = 0.3
        self.marker_publisher.publish(marker)


def main():
    rospy.init_node('bin_marker_cropper')
    from arm_demo import wait_for_time
    wait_for_time()

    cropper = BinMarkerCropper()
    rospy.loginfo('spinning...')
    rospy.spin()


if __name__ == "__main__":
    main()