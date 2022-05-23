#!/usr/bin/env python3
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy
COL_WIDTH_0 = .305
ROW_HEIGHT_0 = 0.19
COL_WIDTH = 0.15
ROW_HEIGHT = 0.127
DEPTH = 0.28
X = 0
Y = 1
Z = 2
OFFSET_X = 0.032
OFFSET_Y = 0.032
TAG_POSE_TOPIC = '/ar_pose_marker'
class BinMarker(object):



    def __init__(self):
        self.marker_publisher = rospy.Publisher('bin_markers', Marker, queue_size=1)
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        rospy.loginfo('bin marker cropper initialized.')
    
    def _tag_pose_callback(self, msg):
        ''' Callback for AR Tag pose publisher, msg contains information
        about the location of each AR tag.
        '''
        self.markers = msg.markers
        # rospy.loginfo(f'found {len(self.markers)} markers.')
        col = rospy.get_param("bin_marker_col", 0)
        row = rospy.get_param("bin_marker_row", 0)
        self.display_bin(int(col),int(row))
    
    @staticmethod
    def get_shelf_points(col: int, row: int):
        left_top_front = BinMarker.get_left_top_front_point(col, row)
        right_bottom_back = left_top_front.copy()
        right_bottom_back[X] += COL_WIDTH_0 if col == 0 else COL_WIDTH
        right_bottom_back[Y] += ROW_HEIGHT_0 if col == 0 else ROW_HEIGHT
        right_bottom_back[Z] += DEPTH
        return [
            left_top_front,
            right_bottom_back,
        ]

    @staticmethod
    def get_left_top_front_point(col, row):
        if col == 0:
            return [0, ROW_HEIGHT_0 * row, 0]
        elif 1 <= col and col <= 3:
            x = (col - 1) * COL_WIDTH + COL_WIDTH_0
            y = row * ROW_HEIGHT
            z = 0
            return [x, y, z]
        elif col == 4:
            x = 0
            y = row * ROW_HEIGHT
            z = 0
            return [x, y, z]
        else:
            return [-COL_WIDTH_0,-ROW_HEIGHT_0,0]

    @staticmethod
    def transform_points(points):
        results = []
        for point in points:
            results.append(BinMarker.transform2ar(point))
        return results

    @staticmethod
    def transform2ar(point):
        x = -point[Y] + OFFSET_X
        y = -point[X] + OFFSET_Y
        z = -point[Z]
        return [x,y,z]

    @staticmethod
    def get_min_max(col, row):
        points = BinMarker.get_shelf_points(col, row)
        points = BinMarker.transform_points(points)
        points = np.array(points)
        mins = points.min(axis=0)
        maxes = points.max(axis=0)
        return np.append(mins, maxes)
        
    def display_bin(self, col, row, rgb=(0,0,1)):
        result = BinMarker.get_min_max(col, row)
        min_x, min_y, min_z, max_x, max_y, max_z = result
        marker = Marker(type=Marker.CUBE)
        marker.header.frame_id = "ar_marker_15"
        marker.pose = Pose()
        marker.pose.position.x = (max_x + min_x) / 2
        marker.pose.position.y = (max_y + min_y) / 2
        marker.pose.position.z = (max_z + min_z) / 2
        marker.pose.orientation.w = 1
        marker.scale = Vector3()
        marker.scale.x = .14
        marker.scale.y = .24
        marker.scale.z = .24
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.3
        self.marker_publisher.publish(marker)
        if col == 0:
            frame = "ar_marker_15"
        elif 1 <= col and col <= 3:
            frame = "ar_marker_15"
        elif col == 4:
            frame = "ar_marker_15"
        rospy.set_param("crop_frame_id", frame)
        rospy.set_param("crop_min_x", float(min_x))    
        rospy.set_param("crop_min_y", float(min_y))
        rospy.set_param("crop_min_z", float(min_z))
        rospy.set_param("crop_max_x", float(max_x))
        rospy.set_param("crop_max_y", float(max_y))
        rospy.set_param("crop_max_z", float(max_z))




def main():
    rospy.init_node('bin_marker')
    from arm_demo import wait_for_time
    wait_for_time()

    marker = BinMarker()
    rospy.loginfo('spinning...')
    rospy.spin()


if __name__ == "__main__":
    main()