POSE_TOPIC = '/amcl_pose'
GOAL_TOPIC = '/move_base_simple/goal'

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

class InteractivePoseMarker:
        def __init__(self, name, x, y, z, text, callback):
            self.server = InteractiveMarkerServer(name)
            self._createIntMarker(x, y, z, text)
            self._makeMarker()
            self.btn_ctrl = InteractiveMarkerControl()
            self.btn_ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
            self.btn_ctrl.always_visible = True
            self.btn_ctrl.markers.append(self._box_marker)
            self._marker.controls.append(self.btn_ctrl)
            self.server.insert(self._marker, callback)
            self.server.applyChanges()

        def _createIntMarker(self, x, y, z, text):
            self._marker = InteractiveMarker()
            self._marker.header.frame_id = "map"
            self._marker.name = f"{text}_marker"
            self._marker.description = text
            self._marker.pose.position.x = x
            self._marker.pose.position.y = y
            self._marker.pose.position.z = z
            self._marker.pose.orientation.w = 1

        def _makeMarker(self):
            self._box_marker = Marker()
            self._box_marker.type = Marker.CUBE
            self._box_marker.pose.orientation.w = 1
            self._box_marker.scale.x = 0.45
            self._box_marker.scale.y = 0.45
            self._box_marker.scale.z = 0.45
            self._box_marker.color.r = 0.0
            self._box_marker.color.g = 0.5
            self._box_marker.color.b = 0.5
            self._box_marker.color.a = 1.0


def callback():
    print("this is the callback")

class NavGoal(object):
    '''
    Sends navigation goals to the robot
    '''
    _latest_pose: PoseWithCovarianceStamped = None
    _locations = {}
    _markers = {}

    def __init__(self):
        # Create a subscriber
        self._pose_subscriber = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, callback=self._pose_callback)
        self._goal_publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)

    def save_current_pose(self, name: str):
        '''
        Save the most recent pose of the robot as a new location with name 
        '''
        msg = PoseStamped()
        msg.header = self._latest_pose.header
        msg.pose = self._latest_pose.pose.pose
        self._locations[name] = msg
        # x, y, z = msg.pose.x, msg.pose.y, msg.pose.z
        marker = InteractivePoseMarker(name, 0, 0, 0, name, callback)

    def get_locations(self):
        '''
        Return a list of locations the user has created
        '''
        return list(self._locations.keys())

    def goto(self, name: str) -> bool:
        '''
        Go to the location of name N, returns false if cannot find name
        '''
        if name in self._locations:
            self._goal_publisher.publish(self._locations[name])
            return True
        else:
            return False


    def delete(self, name) -> bool:
        '''
        Deletes the location with name N, returns false if cannot find name
        '''
        if name not in self._locations:
            return False
        self._locations.pop(name)
        return True

    def _pose_callback(self, msg):
        '''
        Called whenever the robot's pose is updated, records the most recent
        pose
        '''
        self._latest_pose = msg