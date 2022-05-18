#! /usr/bin/env python3

from time import sleep
import robot_api
import rospy
from geometry_msgs.msg import PoseStamped
from perception_msgs.msg import ObjectPose
from ar_track_alvar_msgs.msg import AlvarMarkers

TAG_POSE_TOPIC = '/ar_pose_marker'
OBJECT_POSE_TOPIC = "/object_pose"
PRE_PICKUP_DIST = 0.2

START_POSE = PoseStamped()
START_POSE.header.frame_id = "base_link"
START_POSE.pose.position.x = 0.4323596954345703
START_POSE.pose.position.y = 0.09097349643707275
START_POSE.pose.position.z = 0.9650132656097412
START_POSE.pose.orientation.x = 0
START_POSE.pose.orientation.y = 0
START_POSE.pose.orientation.z = 0
START_POSE.pose.orientation.w = 1

DROP_1 = PoseStamped()
DROP_1.header.frame_id = "base_link"
DROP_1.pose.position.x = 0.5148853659629822
DROP_1.pose.position.y = 0.011607034131884575
DROP_1.pose.position.z = 0.8320705890655518
DROP_1.pose.orientation.x = 0.0021663254592567682
DROP_1.pose.orientation.y = 0.26713114976882935
DROP_1.pose.orientation.z = -0.009311444126069546
DROP_1.pose.orientation.w = 0.9636127352714539


DROP_2 = PoseStamped()
DROP_2.header.frame_id = "base_link"
DROP_2.pose.position.x = 0.5997003316879272
DROP_2.pose.position.y = 0.009853675961494446
DROP_2.pose.position.z = 0.6659631133079529
DROP_2.pose.orientation.x = 0.0032368043903261423
DROP_2.pose.orientation.y = 0.37761861085891724
DROP_2.pose.orientation.z = -0.008995501324534416
DROP_2.pose.orientation.w = 0.9259118437767029


DROP_3 = PoseStamped()
DROP_3.header.frame_id = "base_link"
DROP_3.pose.position.x = 0.7164484858512878
DROP_3.pose.position.y = 0.007487976923584938
DROP_3.pose.position.z = 0.4970704913139343
DROP_3.pose.orientation.x = 0.005603693891316652
DROP_3.pose.orientation.y = 0.6194995641708374
DROP_3.pose.orientation.z = -0.007745636161416769
DROP_3.pose.orientation.w = 0.7849387526512146

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class Pickup():

    objects = []

    def __init__(self, silent=False) -> None:
        self.silent = silent
        self.arm = robot_api.Arm()
        self.gripper = robot_api.Gripper()
        rospy.Subscriber(TAG_POSE_TOPIC, AlvarMarkers, self._tag_pose_callback)
        rospy.Subscriber(OBJECT_POSE_TOPIC, ObjectPose, self._get_pose)

    def _tag_pose_callback(self, msg):
        pass

    def _get_pose(self, msg):
        self.objects.clear()
        for name, pose in zip(msg.names, msg.poses):
            self.objects.append({
                "name": name,
                "pose": pose
            })
        if not self.silent:
            rospy.loginfo(f"Sees {len(self.objects)} objects")
    
    def pickup(self, name):
        for object in self.objects:
            if object['name'] == name:
                if not self.silent:
                    rospy.loginfo(f"Picking up {name} at {object['pose']}")
                pose = object['pose']
                self.gripper.open()
                pose.pose.position.z += 0.166 + PRE_PICKUP_DIST
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = .707
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = .707
                if not self.silent:
                    rospy.loginfo(f"{pose}")
                self.arm.move_to_pose_ik(pose)
                pose.pose.position.z -= PRE_PICKUP_DIST
                self.arm.move_to_pose_ik(pose)
                self.gripper.close(max_effort=60)
                pose.pose.position.z += PRE_PICKUP_DIST
                self.arm.move_to_pose_ik(pose)
                self.drop()
                return True
        if not self.silent:
            rospy.loginfo(f"Couldn't find object {name}")
        return False

    def drop(self):
        if not self.silent:
            rospy.loginfo("Starting Drop")
        self.arm.move_to_pose_ik(START_POSE)
        if not self.silent:
            rospy.loginfo("Moving to Drop 1")
        self.arm.move_to_pose_ik(DROP_1)
        if not self.silent:
            rospy.loginfo("Moving to Drop 2")
        self.arm.move_to_pose_ik(DROP_2)
        if not self.silent:
            rospy.loginfo("Moving to Drop 3")
        self.arm.move_to_pose_ik(DROP_3)
        self.gripper.open()
        if not self.silent:
            rospy.loginfo("Finished Drop")


def main():
    rospy.init_node('pickup_demo')
    wait_for_time()
    pickup = Pickup()
    sleep(1)
    rospy.loginfo(f"Finished initializing pickup")
    pickup.pickup("pill_box")
    pickup.drop()
    rospy.spin()

if __name__ == '__main__':
    main()