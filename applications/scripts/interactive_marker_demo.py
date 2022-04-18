#!/usr/bin/env python3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
# ... Other imports ...
import rospy
import robot_api
import math

def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass

class MvmtInteractiveMarker:
    def __init__(self, name, x, y, z, text, callback):
        self.server = InteractiveMarkerServer(name)
        self.marker = createIntMarker(x, y, z, text)
        self.box_marker = makeMarker()
        self.btn_ctrl = InteractiveMarkerControl()
        self.btn_ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        self.btn_ctrl.always_visible = True
        self.btn_ctrl.markers.append(self.box_marker)
        self.marker.controls.append(self.btn_ctrl)
        self.server.insert(self.marker, callback)
        self.server.applyChanges()


def main():

    base = robot_api.Base()

    def go_forward(input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            rospy.loginfo(input.marker_name + ' was clicked.')
            base.go_forward(0.5)
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')

    def turn_left(input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            rospy.loginfo(input.marker_name + ' was clicked.')
            base.turn(30 * math.pi/180)
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')

    def turn_right(input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            rospy.loginfo(input.marker_name + ' was clicked.')
            base.turn(-30 * math.pi/180)
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')
    rospy.init_node('interactive_marker_node')
        # base.go_forward(0.5)
    forward_marker = MvmtInteractiveMarker("forward", 1, 0, 0, "forward", go_forward)
    left_marker = MvmtInteractiveMarker("left", 1, 1, 0, "left", turn_left)
    right_marker = MvmtInteractiveMarker("right", 1, -1, 0, "right", turn_right)

    rospy.spin()

    
def createIntMarker(x, y, z, text):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = f"{text}_marker"
    int_marker.description = text
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = z
    int_marker.pose.orientation.w = 1
    return int_marker

def makeMarker():
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    return box_marker


if __name__ == '__main__':
  main()