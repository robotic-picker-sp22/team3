#! /usr/bin/python3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
# ... Other imports ...
import rospy
from robot_api import Arm, Gripper
from interactive_markers.menu_handler import MenuHandler, MenuEntry
from geometry_msgs.msg import PoseStamped
import math

def make_6dof_controls():
    controls = []

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)
    return controls

def gripper_markers(scale=1):
    GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
    L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
    R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
    palm = Marker()
    palm.type = Marker.MESH_RESOURCE
    palm.mesh_resource = GRIPPER_MESH
    palm.pose.position.x = 0.166
    lf = Marker()
    lf.type = Marker.MESH_RESOURCE
    lf.mesh_resource = L_FINGER_MESH
    lf.pose.position.x = 0.166
    lf.pose.position.y = -.05
    rf = Marker()
    rf.type = Marker.MESH_RESOURCE
    rf.mesh_resource = R_FINGER_MESH
    rf.pose.position.x = 0.166
    rf.pose.position.y = .05
    markers = [palm, lf, rf]
    for m in markers:
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = 0
        m.color.g = 1.0
        m.color.b = 0
        m.color.a = 1.0
    return markers

def callback_now():
    pass

class GripperTeleop(object):
    def __init__(self, arm: Arm, gripper: Gripper, im_server: InteractiveMarkerServer):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def _menu_entries(self):
        move = MenuEntry()
        move.id = 1
        move.title = "Move Gripper Here"
        open = MenuEntry()
        open.id = 2
        open.title = 'Open gripper'
        close = MenuEntry()
        close.id = 3
        close.title = 'Close gripper'
        items = [move, open, close]
        for i in items:
            i.parent_id = 0
            i.command_type = MenuEntry.FEEDBACK
        return items


    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = 'base_link'
        gripper_im.name = 'gripper'
        gripper_im.scale = 0.25  # make controls smaller
        gripper_im.pose.position.x = 0.400
        gripper_im.pose.position.y = -0.300
        gripper_im.pose.position.z = 0.800
        gripper_im.pose.orientation.w = 1
        # Display gripper
        gripper_ctrl = InteractiveMarkerControl()
        gripper_ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        gripper_ctrl.always_visible = True
        gripper_ctrl.markers.extend(gripper_markers())
        gripper_im.controls.append(gripper_ctrl)
        # Get 6DOF controlls and append to interactive marker
        controls = make_6dof_controls()
        gripper_im.controls.extend(controls)
        # Add menu entries
        gripper_im.menu_entries.extend(self._menu_entries())
        # Add gripper IM to server
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        rospy.loginfo("applying to menu handler.")

    def handle_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            idx = feedback.menu_entry_id
            if idx == 1:
                rospy.loginfo('move')
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'base_link'
                pose_stamped.pose = self._im_server.get('gripper').pose
                kwargs = {
                    'allowed_planning_time': 15,
                    'execution_timeout': 10,
                    'num_planning_attempts': 5,
                    'replan': False
                }
                error = self._arm.move_to_pose(pose_stamped, **kwargs)
                if error is not None:
                    rospy.logerr('Pose failed: {}'.format(error))
                else:
                    rospy.loginfo('Pose succeeded')
            elif idx == 2:
                rospy.loginfo('open')
                self._gripper.open()
            elif idx == 3:
                rospy.loginfo('close')
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            im = self._im_server.get('gripper')
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose = im.pose
            ok = self._arm.compute_ik(pose_stamped, print=False)
            if ok:
                rospy.loginfo("ik ok")
            else:
                rospy.loginfo('ik bad')
            markers = im.controls[0].markers
            for m in markers:
                if ok:
                    m.color.g = 1.0
                    m.color.r = 0.0
                else:
                    m.color.g = 0.0
                    m.color.r = 1.0
            self._im_server.insert(im, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()

            


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server: InteractiveMarkerServer):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_teleop')
    from arm_demo import wait_for_time
    wait_for_time()
    arm = Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    gripper = Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.loginfo('spinning...')
    rospy.spin()


if __name__ == "__main__":
    main()