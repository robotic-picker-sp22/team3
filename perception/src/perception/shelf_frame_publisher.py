#! /usr/bin/env python3

'''
This file uses the AR markers to publish a marker for the entire shelf. 

Published axes:
x - left and right on the shelf
y - up and down on the shelf
z - into and out of the shelf
'''
# Link to tutorial:
# http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29

import rospy
import math
import tf2_msgs.msg
import tf.transformations as tft
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from bin_cropper import SHELF_FRAME_NAME
import robot_api

USE_SIM_SETTINGS = False
TAG_POSE_TOPIC = '/ar_pose_marker'
TF_TOPIC = '/tf'


# Names of the three expected markers on the shelf
EXPECTED_MARKERS = [15, 5, 4]
# EXPECTED_MARKERS = [15]

if USE_SIM_SETTINGS:
# The distance from the markers to the shelf edge
MARKER_15_DIST = 0.02
MARKER_15_TO_5_DIST = 0.2209874427225697
    MARKER_5_DIST = MARKER_15_DIST + MARKER_15_TO_5_DIST
    MARKER_5_TO_4_DIST = 0.35778027901128795
    MARKER_4_DIST = MARKER_5_DIST + MARKER_5_TO_4_DIST
DEFAULT_MARKER_DIST = {
            EXPECTED_MARKERS[0]: MARKER_15_DIST,
            EXPECTED_MARKERS[1]: MARKER_5_DIST,
            EXPECTED_MARKERS[2]: MARKER_4_DIST
    } # TODO: this will have to be tuned
else:
    # Distances between markers on the shelf
    MARKER_15_DIST = 0.035
    MARKER_15_TO_5_DIST = 0.4079494706274665
    MARKER_5_DIST = MARKER_15_DIST + MARKER_15_TO_5_DIST
    MARKER_5_TO_4_DIST = 0.35778027901128795
    MARKER_4_DIST = MARKER_5_DIST + MARKER_5_TO_4_DIST
    # Distance from each marker to the left corner of the shelf
    DEFAULT_MARKER_DIST = {
            EXPECTED_MARKERS[0]: MARKER_15_DIST,
            EXPECTED_MARKERS[1]: MARKER_5_DIST,
            EXPECTED_MARKERS[2]: MARKER_4_DIST
        }


# Default angle offsets from base_link
DEFAULT_ROLL = math.radians(0)
DEFAULT_PITCH = math.radians(0)
# The height of the shelf off the ground
DEFAULT_SHELF_HEIGHT = 1.52 # TODO: tune this

def calculate_shelf_angle(x1, y1, x2, y2):
    ''' Takes two points and returns the angle of the line through the points
    '''
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    angle = math.atan(dy/dx)
    if (x1 < x2):
        angle = math.radians(180) - angle
    return angle

def translate_point(x1, y1, angle, distance):
    ''' Takes a point, angle, and distance then returns a new translated point
    '''
    x_off = distance * math.cos(angle)
    y_off = distance * math.sin(angle)
    x_end = x1+x_off
    y_end = y1+y_off
    # rospy.loginfo(f"Translated ({x1}, {y1}) by {round(math.degrees(angle), 4)} deg and dist {round(distance, 4)} to ({x_end}, {y_end})")
    return x1+x_off, y1+y_off


def print_marker_distances(markers):
    ''' Prints the distances between the markers
    '''
    if len(markers) < 1:
        rospy.loginfo("No markers to print distance.")
        return
    elif len(markers) < 2:
        rospy.loginfo(f'Only one marker {markers[0].id}')
        return
    rospy.loginfo(f'Printing distances for markers {[m.id for m in markers]}')
    prev_id = markers[0].id
    prev_x = markers[0].pose.pose.position.x
    prev_y = markers[0].pose.pose.position.y
    for i in range(1, len(markers)):
        x = markers[i].pose.pose.position.x
        y = markers[i].pose.pose.position.y
        dist = math.sqrt((x-prev_x)**2 + (y-prev_y)**2)
        rospy.loginfo(f"{dist} between markers {prev_id} and {markers[i].id}")
        prev_x, prev_y = x, y
        prev_id = markers[i].id

def quaternion_from_ori(orientation):
    return (orientation.x, orientation.y, orientation.z, orientation.w)
    

class ShelfMarkerBroadcaster:

    # The current tranform from base_link to the shelf frame
    frame_transform: geometry_msgs.msg.Transform()

    # The ammount of error between the angle of the shelf and the markers
    angle_offsets = {
        EXPECTED_MARKERS[0]: 0,
        EXPECTED_MARKERS[1]: 0,
        EXPECTED_MARKERS[2]: 0
    }

    markers = []
    
    def __init__(self):
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.frame_transform = None
        rospy.loginfo("Waiting for markers...")
        while len(self.markers) < 1:
            rospy.sleep(0.5)
        rospy.loginfo("Found markers, calculating transform...")
        while self.frame_transform is None:
            rospy.sleep(0.5)
        rospy.loginfo("Calculated transform, publishing pose...")
        while not rospy.is_shutdown():
            # Run at 2Hz
            rospy.sleep(0.5)
            self._publish_pose()

    def _tag_pose_callback(self, msg):
        ''' Callback for AR Tag pose publisher, msg contains information
        about the location of each AR tag.
        '''
        markers = []
        for marker in msg.markers:
            if marker.id in EXPECTED_MARKERS:
                markers.append(marker)
            else:
                rospy.logerr(f'Found unexpected marker {marker.id}')
        if len(markers) > 1:
            # Sort the markers into the expected order left to right
            f = lambda m : EXPECTED_MARKERS.index(m.id)
            markers.sort(key=f)
        self.markers = markers
        print_marker_distances(markers) #TODO: remove this once the distances are tuned
        self._update_transform()
    

    def _update_transform(self):
        ''' Uses the AR markers to update the transform value
        '''
        if len(self.markers) == 0:
            return
        if len(self.markers) == 1:
            marker = self.markers[0]
            rospy.logerr(f'Can only see one marker {marker.id}')
            return
            # rospy.loginfo(f'Using only marker {marker.id}')
            # x1 = marker.pose.pose.position.x
            # y1 = marker.pose.pose.position.y
            # quaternion = quaternion_from_ori(marker.pose.pose.orientation)
            # euler = tft.euler_from_quaternion(quaternion)
            # shelf_angle = euler[2]
            # rospy.loginfo(f'Found shelf_angle {shelf_angle}')
            # rospy.loginfo(f"Marker rotated at {euler[0]} {euler[1]} {euler[2]}")
            # distance = DEFAULT_MARKER_DIST[marker.id]
            # x, y = translate_point(x1, y1, shelf_angle, distance)
        else:
            # Use the left and right markers
            left = self.markers[0]
            right = self.markers[-1]
            rospy.loginfo(f'Using markers {left.id} and {right.id}')
            x1 = left.pose.pose.position.x
            y1 = left.pose.pose.position.y
            x2 = right.pose.pose.position.x
            y2 = right.pose.pose.position.y
            shelf_angle = calculate_shelf_angle(x1, y1, x2, y2)
            self._update_marker_transforms(shelf_angle)
            x, y = self._calculate_shelf_position(shelf_angle)
        rospy.loginfo(f'Found shelf angle of {shelf_angle} rad {math.degrees(shelf_angle)} deg')
        self._set_transform(x, y, shelf_angle)
        self._update_marker_transforms(shelf_angle)
    

    def _update_marker_transforms(self, shelf_angle):
        ''' Given the actual angle of the shelf, update the rotation errors
        for each AR marker
        '''
        # Get the transform matrix from the base link to the shelf frame
        
        for marker in self.markers:
            # Get the marker transform
            pass


    def _calculate_shelf_position(self, shelf_angle):
        ''' Calculates the position of the corner of the shelf given the angle
        of the shelf
        '''
        xs = []
        ys = []
        for marker in self.markers:
            m_x = marker.pose.pose.position.x
            m_y = marker.pose.pose.position.y
            distance = DEFAULT_MARKER_DIST[marker.id]
            x, y = translate_point(m_x, m_y, shelf_angle, distance)
            xs.append(x)
            ys.append(y)
        uncertain_x = max(xs) - min(xs)
        uncertain_y = max(ys) - min(ys)
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        rospy.loginfo(f'Calculated a shelf position of ({round(avg_x, 3)}, {round(avg_y, 3)}) with uncertainty: x={round(uncertain_x, 5)} y={round(uncertain_y, 5)}')
        return avg_x, avg_y
        
            
    def _set_transform(self, x, y, angle):
        ''' Takes the xy position of the shelf and the angle, and updates the 
        transform from the base_link to the shelf frame
        '''
        self.frame_transform = geometry_msgs.msg.Transform()
        quaternion = tft.quaternion_from_euler(0, 0, angle)
        # TODO: remove the below
        # quaternion = quaternion_from_ori(self.markers[0].pose.pose.orientation)
        # euler = tft.euler_from_quaternion(quaternion)
        # rospy.loginfo(f'Euler2={euler[2]} angle={angle} ({round(math.degrees(angle), 4)} degrees)')
        # euler = (0, 0, angle)
        # quaternion = tft.quaternion_from_euler(euler[0], euler[1], euler[2])
        self.frame_transform.translation.x = x
        self.frame_transform.translation.y = y
        self.frame_transform.translation.z = rospy.get_param("shelf_height", DEFAULT_SHELF_HEIGHT)
        self.frame_transform.rotation.x = quaternion[0]
        self.frame_transform.rotation.y = quaternion[1]
        self.frame_transform.rotation.z = quaternion[2]
        self.frame_transform.rotation.w = quaternion[3]


    def _publish_pose(self):
        ''' Publishes the most updated translation from the base_link to the 
        shelf frame
        '''
        if self.frame_transform is None:
            rospy.logerr("No transformation")
            return
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = SHELF_FRAME_NAME
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 2.0
        # t.transform.translation.z = 0.0

        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0
        t.transform = self.frame_transform

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)
        rospy.loginfo('Published new marker')
   

if __name__ == '__main__':
    rospy.init_node('shelf_marker_broadcaster')
    from bin_cropper import wait_for_time
    wait_for_time()
    smb = ShelfMarkerBroadcaster()

    rospy.spin()