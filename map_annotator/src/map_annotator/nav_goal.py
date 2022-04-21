POSE_TOPIC = '/amcl_pose'
GOAL_TOPIC = '/move_base_simple/goal'

import random
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import pickle
import os

DEFAULT_FILE = 'navgoal_data.pkl'
TOPIC = "/map_annotator/map_poses"
class NavGoal(object):
    '''
    Stores a list of poses (locations in the map/room) and names for those 
    locations. The robot can be moved to one of the saved locations. The 
    locations are displayed as markers in RViz, where the user can change
    the position and orientation.   

    Refrence code:
    InteractiveMarkerServer code: http://docs.ros.org/en/jade/api/interactive_markers/html/interactive__marker__server_8py_source.html
    '''
    _latest_pose: PoseWithCovarianceStamped = None
    _names = set()

    def __init__(self, pkl_path=None):
        '''
        Create a subscriber to the robot location, a publisher to set goal poses for the robot to go to, 
        an interactive marker server to create markers in RViz, and load previous names and poses from a
        pickle file on startup
        '''
        self._pose_subscriber = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, callback=self._pose_callback)
        self._goal_publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)
        self._marker_server = InteractiveMarkerServer(TOPIC)
        self._pkl_path = pkl_path if pkl_path is not None else os.path.join(rospkg.RosPack().get_path('map_annotator'), DEFAULT_FILE)
        self._load_from_file()

    def __del__(self):
        '''
        On exit, save names and poses to a pickel file on disk, so the same 
        data can be restored on startup
        '''
        self._save_to_file()

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

    def get_locations(self) -> list:
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

    def _load_from_file(self):
        '''
        Load previous names and poses from the file and create new 
        interactive markers for them
        '''
        if os.path.isfile(self._pkl_path):
            rospy.loginfo(f'Found data file {self._pkl_path}')
            file = open(self._pkl_path, 'rb')
            data = pickle.load(file)
            for name in data.keys():
                self._names.add(name)
                self._create_interactive_marker(name, data[name])
            file.close()


    def _save_to_file(self):
        '''
        Save all the names and poses into a file on disk
        '''
        file = open(self._pkl_path, 'wb')
        data = {}
        for name in self._names:
            marker = self._marker_server.get(name)
            data[name] = marker.pose
        pickle.dump(data, file)
        file.close()
        rospy.loginfo(f'Saved navigation pose data to {self._pkl_path}')


    def _pose_callback(self, msg):
        '''
        Called whenever the robot's pose is updated, records the most recent
        pose
        '''
        self._latest_pose = msg

    def _create_interactive_marker(self, marker_name: str, pose=None):
        '''
        Creates a new arrow interactive marker

        marker_name - the text to display above the marker
        pose - the pose orientation and location of the marker; origin if None
        '''
        # Create the interactive marker
        new_imarker = InteractiveMarker()
        new_imarker.header.frame_id = 'map'
        new_imarker.name = marker_name
        new_imarker.description = marker_name
        if pose is not None:
            new_imarker.pose = pose
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
        '''
        Called when user interacts with marker in RViz
        '''
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        # rospy.loginfo(s)

    @staticmethod
    def _create_arrow_marker():
        '''
        Creates the vizual arrow marker for the interactive marker
        returns: a new arrow marker
        '''
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
        '''
        Creates the move and rotate controls for the interactive marker
        returns: control to move along the xy plane, control to rotate around z axis
        '''
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