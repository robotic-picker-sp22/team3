#!/usr/bin/env python3

import rospy
import rospy
from geometry_msgs.msg import Vector3, Pose
from visualization_msgs.msg import Marker
from perception.shelf_frame_publisher import SHELF_FRAME_NAME

# Name of the new frame for the shelf

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

# '''
# Only works if cropper is already running:
# rosrun perception point_cloud_demo labels/

# For real robot:
# rosrun perception point_cloud_demo labels/ cloud_in:=/head_camera/depth_registered/points
# '''

# TAG_POSE_TOPIC = '/ar_pose_marker'

# """
# Shelf map
# +--------+----+----+----+----+
# |        | 10 | 20 | 30 | 40 |
# |   00   |    |    |    |    |
# |        +----+----+----+----+
# +--------+ 07 | 17 | 27 | 37 |
# |        |    |    |    |    |
# |   01   +----+----+----+----+
# |        | 08 | 18 | 28 | 38 |
# +--------+    |    |    |    |
# |        +----+----+----+----+
# |   02   | 09 | 19 | 29 | 39 |
# |        |    |    |    |    |
# +--------+----+----+----+----+
# |        | 10 | 20 | 30 | 40 |
# |   03   |    |    |    |    |
# |        +----+----+----+----+
# +--------+ 11 | 21 | 31 | 41 |
# |        |    |    |    |    |
# |   04   +----+----+----+----+
# |        | 12 | 22 | 32 | 42 |
# +--------+    |    |    |    |
# |        +----+----+----+----+
# |   05   | 13 | 23 | 33 | 43 |
# |        |    |    |    |    |
# +--------+----+----+----+----+
#          | 14 | 24 | 34 | 44 |
#          |    |    |    |    |
#          +----+----+----+----+
#          | 15 | 25 | 35 | 45 |
#          |    |    |    |    |
#          +----+----+----+----+

# """

# # relative to the shelf top left front
# crop_bounds: list = [
# #   [min_x, max_x, min_y, max_y, min_z, max_z], # shelf
#     [ 0.00,  0.31,  0.00,  0.22,  0.20, -0.28], # 00
#     [ 0.00,  0.31,  0.23,  0.04,  0.20, -0.28], # 01*
#     [ 0.00,  0.31, -0.18,  0.04,  0.20, -0.28], # 02
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 03
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 04
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 05
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 06
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 07
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 08
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 09
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 10
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 11
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 12
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 13
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 14
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 15
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 16
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 17
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 18
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 19
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 20
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 21
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 22
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 23
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 24
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 25
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 26
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 27
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 28
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 29
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 30
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 31
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 32
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 33
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 34
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 35
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 36
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 37
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 38
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 39
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 40
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 41
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 42
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 43
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28], # 44
#     [-0.32, -0.17, -0.18,  0.04,  0.20, -0.28]  # 45
# ]                                              

# class BinMarkerCropper(object):

#     markers: list = []

#     def __init__(self):
#         rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
#         rospy.loginfo('bin marker cropper initialized.')
#         self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    

#     def _tag_pose_callback(self, msg):
#         ''' Callback for AR Tag pose publisher, msg contains information
#         about the location of each AR tag.
#         '''
#         self.markers = msg.markers
#         # rospy.loginfo(f'found {len(self.markers)} markers.')
#         # shelf: int = int(rospy.get_param('crop_shelf', 1))
#         # self.crop_shelf_for_marker(shelf)

#         # Crop to the shelf below the one with the marker
#         self._crop(-0.25, -0.10, -0.1)


#     # def wait_for_markers(self):
#     #     ''' Hangs until this receives markers for the AR tags.
#     #     '''
#     #     rospy.loginfo('waiting for markers...')
#     #     while len(self.markers) == 0:
#     #         rospy.sleep(0.1)
#     #     rospy.loginfo('received markers.')


#     def _crop(self, x, y, z):
#         rospy.loginfo(f'cropping to ({x}, {y}, {z})')
#         DEFAULT_WID = 0.24
#         DEFAULT_HEI = 0.14
#         DEFAULT_DEP = 0.24
#         min_x = x - DEFAULT_HEI/2
#         min_y = y - DEFAULT_WID/2
#         min_z = z - DEFAULT_DEP/2
#         max_x = x + DEFAULT_HEI/2 
#         max_y = y + DEFAULT_WID/2 
#         max_z = z + DEFAULT_DEP/2 
#         rospy.set_param("crop_min_x", min_x)    
#         rospy.set_param("crop_min_y", min_y)
#         rospy.set_param("crop_min_z", min_z)
#         rospy.set_param("crop_max_x", max_x)
#         rospy.set_param("crop_max_y", max_y)
#         rospy.set_param("crop_max_z", max_z)
#         self._pub_marker(min_x, max_x, min_y, max_y, min_z, max_z)


#     def _crop_dims(self, x, y, z, w, h, d):
#         rospy.loginfo(f'cropping to ({x}, {y}, {z})')
#         y += -0.14
#         z += -0.08
#         DEFAULT_WID = w
#         DEFAULT_HEI = h
#         DEFAULT_DEP = d
#         min_x = x - DEFAULT_DEP/2
#         min_y = y - DEFAULT_WID/2
#         min_z = z - DEFAULT_HEI/2
#         max_x = x + DEFAULT_DEP/2 
#         max_y = y + DEFAULT_WID/2 
#         max_z = z + DEFAULT_HEI/2 
#         rospy.set_param("crop_min_x", min_x)    
#         rospy.set_param("crop_min_y", min_y)
#         rospy.set_param("crop_min_z", min_z)
#         rospy.set_param("crop_max_x", max_x)
#         rospy.set_param("crop_max_y", max_y)
#         rospy.set_param("crop_max_z", max_z)


#     def crop_for_marker(self):
#         if len(self.markers) < 1:
#             rospy.logerr('Could not find any markers!')
#         for marker in self.markers:
#             assert(marker.header.frame_id == '/base_link')
#             p = marker.pose.pose.position
#             self._crop(p.x, p.y, p.z)
#             return
#         rospy.logerr('Failed to find any markers!')

    
#     def crop_shelf_for_marker(self, shelf: int):
#         '''
#         Args:
#             shelf - 
#         '''
#         bounds = self._get_shelf_bounds(shelf)
#         x = bounds[0]
#         w = bounds[1]
#         y = bounds[2]
#         h = bounds[3]
#         z = bounds[4]
#         d = bounds[5]
#         rospy.loginfo(f'cropping to ({x}, {y}, {z}), ({w}, {h}, {d})')
#         x += 0.03
#         y += 0.03
#         max_x = x - w
#         max_y = y - h
#         max_z = z + d
#         rospy.set_param("crop_min_x", max_x)    
#         rospy.set_param("crop_min_y", max_y)
#         rospy.set_param("crop_min_z", z)
#         rospy.set_param("crop_max_x", x)
#         rospy.set_param("crop_max_y", y)
#         rospy.set_param("crop_max_z", max_z)
#         self._pub_marker(x, w, y, h, z, d)

#     def _get_shelf_bounds(self, shelf: int):
#         '''
#         Returns the bounding box for a given shelf
#         Args:
#             shelf: the index of the shelf to get the bounds for 
#         Returns: the min and max x, y, z values for the shelf (6 total values)
#         '''
#         min_x = 0
#         max_x = 0
#         min_y = 0
#         max_y = 0
#         min_z = -0.28
#         max_z = 0.5
#         padding = 0.01
#         if (shelf < 10):
#             min_x = 0
#             max_x = 0.31
#             min_y = shelf * 0.22
#             max_y = 0.22
#         else:
#             row = shelf % 10
#             col = shelf // 10
#             min_x = 0.31 + col * 0.15
#             max_x = 0.15
#             min_y = row * 0.13
#             max_y = 0.13

#         max_x -= padding
#         max_y -= padding
#         return min_x, max_x, min_y, max_y, min_z, max_z

#     def _pub_marker(self, min_x, max_x, min_y, max_y, min_z, max_z):
#         marker = Marker(type=Marker.CUBE)
#         marker.header.frame_id = SHELF_FRAME_NAME
#         # Pose is at the center of the marker
#         marker.pose = Pose()
#         marker.pose.position.x = (max_x + min_x) / 2
#         marker.pose.position.y = (max_y + min_y) / 2
#         marker.pose.position.z = (max_z + min_z) / 2
#         marker.scale = Vector3()
#         marker.scale.x = max_x - min_x
#         marker.scale.y = max_y - min_y
#         marker.scale.z = max_z - min_z
#         marker.color.r = 1
#         marker.color.a = 0.3
#         self.marker_publisher.publish(marker)

# Depth of all bins
BIN_DEPTH = 0.28
DEFAULT_Y = -0.12
# Positions of the columns left and right
COL_POSITIONS = [-0.13, -0.355, -0.495]

LARGE_BIN_HEIGHT = 0.12
LARGE_BIN_WIDTH = 0.246
BIG_ROW_POSITIONS = [-0.105, -0.285, -0.457]  

# Indices of the columns with small bins
SMALL_COLUMNS = [1, 2, 3]
SMALL_BIN_HEIGHT = 0.09
SMALL_BIN_WIDTH = 0.10
SMALL_ROW_POSITIONS = [-0.078, -0.204, -0.325, -0.445]


def crop_to_bin(row: int, col: int):
    x = COL_POSITIONS[col]
    y = DEFAULT_Y
    if col in SMALL_COLUMNS:
        height = SMALL_BIN_HEIGHT
        width = SMALL_BIN_WIDTH
        z = SMALL_ROW_POSITIONS[row]
    else:
        height = LARGE_BIN_HEIGHT
        width = LARGE_BIN_WIDTH
        z = BIG_ROW_POSITIONS[row]
    depth = BIN_DEPTH
    # rospy.loginfo(f'cropping to ({x}, {y}, {z})')
    min_x = x - width/2
    min_y = y - depth/2
    min_z = z - height/2
    max_x = x + width/2 
    max_y = y + depth/2 
    max_z = z + height/2 
    rospy.set_param("crop_min_x", min_x)    
    rospy.set_param("crop_min_y", min_y)
    rospy.set_param("crop_min_z", min_z)
    rospy.set_param("crop_max_x", max_x)
    rospy.set_param("crop_max_y", max_y)
    rospy.set_param("crop_max_z", max_z)
    return x, y, z, width, depth, height
    
    
def pub_marker(x, y, z, width, depth, height):
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    marker = Marker(type=Marker.CUBE)
    marker.header.frame_id = SHELF_FRAME_NAME
    # Pose is at the center of the marker
    marker.pose = Pose()
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale = Vector3()
    marker.scale.x = width
    marker.scale.y = depth
    marker.scale.z = height
    marker.color.r = 1
    marker.color.a = 0.3
    marker_publisher.publish(marker)


def print_usage():
    print('Crops to a certain bin')
    print('Usage: rosrun applications bin_marker_cropper.py row column')

def main():
    rospy.init_node('bin_marker_cropper')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 3:
        print_usage()
        return
    row = int(argv[1])
    col = int(argv[2])
    x, y, z, width, depth, height = crop_to_bin(row, col)
    
    for i in range(3):
        pub_marker(x, y, z, width, depth, height)
        rospy.sleep(0.2)


if __name__ == "__main__":
    main()