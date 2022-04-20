POSE_TOPIC = '/amcl_pose'
GOAL_TOPIC = '/move_base_simple/goal'

import random
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

class NavGoal(object):
    '''
    Sends navigation goals to the robot
    '''
    _latest_pose: PoseWithCovarianceStamped = None
    _names = set()

    def __init__(self):
        # Create a subscriber
        self._pose_subscriber = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, callback=self._pose_callback)
        self._goal_publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)
        self._marker_server = InteractiveMarkerServer('pose_marker')

    def save_current_pose(self, name: str):
        '''
        Save the most recent pose of the robot as a new location with name 
        '''
        msg = PoseStamped()
        msg.header = self._latest_pose.header
        msg.pose = self._latest_pose.pose.pose
        self._names.add(name)
        self._create_interactive_marker(name, msg.pose)

    def create_new_pose(self, name: str):
        '''
        Create a new pose at the origin
        '''
        self._names.add(name)
        self._create_interactive_marker(name)

    def get_locations(self):
        '''
        Return a list of locations the user has created
        '''
        return list(self._names)

    def goto(self, name: str) -> bool:
        '''
        Go to the location of name N, returns false if cannot find name
        '''
        marker = self._marker_server.get(name)
        if marker is None:
            return False

        pose = PoseStamped()
        pose.header = marker.header
        pose.pose = marker.pose
        self._goal_publisher.publish(pose)
        return True


    def delete(self, name: str) -> bool:
        '''
        Deletes the location with name N, returns false if cannot find name
        '''
        out = name in self._names
        self._names.remove(name)
        self._marker_server.erase(name)
        self._marker_server.applyChanges()
        return out


    def _pose_callback(self, msg):
        '''
        Called whenever the robot's pose is updated, records the most recent
        pose
        '''
        self._latest_pose = msg

    def _create_interactive_marker(self, marker_name: str, pose=None):
        # Create the interactive marker
        new_imarker = InteractiveMarker()
        new_imarker.header.frame_id = 'map'
        new_imarker.name = marker_name
        new_imarker.description = marker_name
        if pose is not None:
            new_imarker.pose = pose
            rospy.loginfo(f'Creating at pose {pose}')
        else:
            new_imarker.pose.position.x = 0
            new_imarker.pose.position.y = 0
            new_imarker.pose.position.z = 0.05
        new_imarker.pose.orientation.w = 1
        # Create arrow marker control
        arrow_ctrl =  InteractiveMarkerControl()
        arrow_ctrl.always_visible = True
        arrow_ctrl.markers.append(NavGoal._create_arrow_marker())
        new_imarker.controls.append(arrow_ctrl)
        # Create controls
        move_ctrl, rotate_ctrl = NavGoal._create_controls()
        new_imarker.controls.append(move_ctrl)
        new_imarker.controls.append(rotate_ctrl)
        # Add the marker to the server
        self._marker_server.insert(new_imarker, NavGoal._marker_callback)
        self._marker_server.applyChanges()

    @staticmethod
    def _marker_callback(feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        # rospy.loginfo(s)

    @staticmethod
    def _create_arrow_marker():
        # Create the arrow marker
        new_marker = Marker()
        new_marker.type = Marker.ARROW
        new_marker.scale.x = 1
        new_marker.scale.y = 0.1
        new_marker.scale.z = 0.1
        # Randomize color
        new_marker.color.r = random.random()
        new_marker.color.g = random.random()
        new_marker.color.b = random.random()
        new_marker.color.a = 1.0
        return new_marker

    @staticmethod
    def _create_controls():
        # Create move and rotate controls
        move_ctrl = InteractiveMarkerControl()
        move_ctrl.orientation.w = 1
        move_ctrl.orientation.x = 0
        move_ctrl.orientation.y = 1
        move_ctrl.orientation.z = 0
        move_ctrl.name = 'move_plane'
        move_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # move_ctrl.always_visible = True
        # Create rotate control
        rotate_ctrl = InteractiveMarkerControl()
        rotate_ctrl.name = 'rotate'
        rotate_ctrl.orientation.w = 1
        rotate_ctrl.orientation.x = 0
        rotate_ctrl.orientation.y = 1
        rotate_ctrl.orientation.z = 0
        rotate_ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_ctrl.orientation_mode = InteractiveMarkerControl.FIXED
        return move_ctrl, rotate_ctrl