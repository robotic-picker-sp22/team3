#! /usr/bin/python3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
# ... Other imports ...
import rospy
from robot_api import Arm, Gripper, ArmJoints
from interactive_markers.menu_handler import MenuHandler, MenuEntry
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import numpy as np
from sensor_msgs.msg import JointState


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

def gripper_markers(scale=1, x=0, y=0, z=0):
    GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
    L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
    R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
    palm = Marker()
    palm.type = Marker.MESH_RESOURCE
    palm.mesh_resource = GRIPPER_MESH
    lf = Marker()
    lf.type = Marker.MESH_RESOURCE
    lf.mesh_resource = L_FINGER_MESH
    lf.pose.position.y = -.05
    rf = Marker()
    rf.type = Marker.MESH_RESOURCE
    rf.mesh_resource = R_FINGER_MESH
    rf.pose.position.y = .05
    markers = [palm, lf, rf]
    for m in markers:
        m.pose.position.x += x
        m.pose.position.y += y
        m.pose.position.z += z
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = 0
        m.color.g = 1.0
        m.color.b = 0
        m.color.a = 1.0
    return markers


def menu_entries():
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


class GripperTeleop(object):
    def __init__(self, arm: Arm, gripper: Gripper, im_server: InteractiveMarkerServer):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server


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
        gripper_ctrl.markers.extend(gripper_markers(x=0.166))
        gripper_im.controls.append(gripper_ctrl)
        # Get 6DOF controlls and append to interactive marker
        controls = make_6dof_controls()
        gripper_im.controls.extend(controls)
        # Add menu entries
        gripper_im.menu_entries.extend(menu_entries())
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
                rospy.loginfo(f"Moving to pose: {pose_stamped}")
                # kwargs = {
                #     'allowed_planning_time': 15,
                #     'execution_timeout': 40,
                #     'num_planning_attempts': 5,
                #     'replan': False
                # }
                # error = self._arm.move_to_pose(pose_stamped, **kwargs)
                # if error is not None:
                #     rospy.logerr('Pose failed: {}'.format(error))
                # else:
                #     rospy.loginfo('Pose succeeded')
                self._arm.move_to_pose_ik(pose_stamped)
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
            # print(pose_stamped)
            ok = self._arm.compute_ik(pose_stamped, print=False)
            # if ok:
            #     rospy.loginfo("ik ok")
            # else:
            #     rospy.loginfo('ik bad')
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
        gripper_ctrl.markers.extend(gripper_markers(x=-0.2))  # pre-grasp
        gripper_ctrl.markers.extend(gripper_markers())  # grasp
        gripper_ctrl.markers.extend(gripper_markers(x=-0.2, z = 0.2))  # lift
        gripper_im.controls.append(gripper_ctrl)
        # Get 6DOF controlls and append to interactive marker
        controls = make_6dof_controls()
        gripper_im.controls.extend(controls)
        # Add menu entries
        gripper_im.menu_entries.extend(menu_entries())
        # Add gripper IM to server
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        rospy.loginfo("applying to menu handler.")

    def _get_poses(self):
        wrist_link_pose = self._im_server.get('gripper').pose
        # transform matrix from wrist_link to base_link
        quaternion = [
            wrist_link_pose.orientation.x,
            wrist_link_pose.orientation.y,
            wrist_link_pose.orientation.z,
            wrist_link_pose.orientation.w
        ]
        matrix = tft.quaternion_matrix(quaternion=quaternion)
        # Offsets from wrist_link_pose
        grip = [-0.166, 0, 0.0, 1]
        pre =  [-0.366, 0, 0.0, 1]
        lift = [-0.366, 0, 0.2, 1]
        # Translate offsets from wrist_link frame to base_link frame
        poses = []
        for wl_pose in [pre, grip, lift]:
            bl_pose = np.dot(matrix, wl_pose)
            x, y, z = bl_pose[:3]
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose.position.x = x + wrist_link_pose.position.x
            pose_stamped.pose.position.y = y + wrist_link_pose.position.y
            pose_stamped.pose.position.z = z + wrist_link_pose.position.z
            pose_stamped.pose.orientation = wrist_link_pose.orientation
            poses.append(pose_stamped)
        return poses

    def _handle_move(self):
        # def move_to_pose(pose_stamped: PoseStamped, oc: OrientationConstraint = None):
        #     kwargs = {
        #         'allowed_planning_time': 10,
        #         'execution_timeout': 20,
        #         'num_planning_attempts': 10,
        #         'tolerance': 0.1,
        #         'replan': True
        #     }   
        #     error = self._arm.move_to_pose(pose_stamped, orientation_constraint=oc, **kwargs)
        #     if error is not None:
        #         rospy.logerr('Pose failed: {}'.format(error))
        #     else:
        #         rospy.loginfo('Pose succeeded')
        #     rospy.sleep(2)

        rospy.loginfo('move')
        
        pre, grip, lift = self._get_poses()
        
        # Move to pre-grip unconstrained
        self._gripper.open()
        # move_to_pose(pre)
        self._arm.move_to_pose_ik(pre)
        # rospy.sleep(3)
        # Move to grip constrained by plane
        # oc = OrientationConstraint()
        # oc.header.frame_id = 'base_link'
        # oc.link_name = 'wrist_roll_link'
        # oc.orientation = pre.pose.orientation
        # oc.absolute_x_axis_tolerance = 0.2
        # oc.absolute_y_axis_tolerance = 0.2
        # oc.absolute_z_axis_tolerance = 3.14
        # oc.weight = 1.0
        # move_to_pose(grip, oc=oc)
        self._arm.move_to_pose_ik(grip)
        self._gripper.close()
        self._arm.move_to_pose_ik(pre)
        # rospy.sleep(2)
        rospy.sleep(1)
        # Move to lift constrained by object orientation
        # move_to_pose(lift, oc=oc)
        self._arm.move_to_pose_ik(lift)

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            idx = feedback.menu_entry_id
            if idx == 1:
                self._handle_move()
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
            for i, pose_stamped in enumerate(self._get_poses()):   
                ok = self._arm.compute_ik(pose_stamped, print=False)
                markers = im.controls[0].markers[i*3:i*3+3]
                for m in markers:
                    if ok:
                        m.color.g = 1.0
                        m.color.r = 0.0
                    else:
                        m.color.g = 0.0
                        m.color.r = 1.0
                self._im_server.insert(im, feedback_cb=self.handle_feedback)
                self._im_server.applyChanges()


def main():
    rospy.init_node('gripper_teleop')
    from arm_demo import wait_for_time
    wait_for_time()
    arm = Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    gripper = Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.loginfo('spinning...')
    rospy.spin()


if __name__ == "__main__":
    main()