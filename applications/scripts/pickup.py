#! /usr/bin/env python3

from time import sleep
import robot_api
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from perception_msgs.msg import ObjectPose
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf.transformations as tft

TAG_POSE_TOPIC = '/ar_pose_marker'
OBJECT_POSE_TOPIC = "/object_pose"
PRE_PICKUP_DIST = 0.2

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class Pickup():

    objects = []

    def __init__(self) -> None:
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
        rospy.loginfo(f"Sees {len(self.objects)} objects")
    
    def pickup(self, name):
        for object in self.objects:
            if object['name'] == name:
                rospy.loginfo(f"Picking up {name} at {object['pose']}")
                pose = object['pose']
                self.gripper.open()
                pose.pose.position.z += 0.166 + PRE_PICKUP_DIST
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = .707
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = .707
                #-0.7525881647719995 -0.005108629837268916 0.6579435716583977 0.026366885665383956
                # pose.pose.orientation.w = 1
                rospy.loginfo(f"{pose}")
                self.arm.move_to_pose_ik(pose)
                pose.pose.position.z -= PRE_PICKUP_DIST
                self.arm.move_to_pose_ik(pose)
                self.gripper.close(max_effort=60)
                pose.pose.position.z += PRE_PICKUP_DIST
                self.arm.move_to_pose_ik(pose)
                break

def main():
    rospy.init_node('pickup_demo')
    wait_for_time()
    pickup = Pickup()
    sleep(3)
    rospy.loginfo(f"Finished initializing pickup")
    pickup.pickup("pill_box")
    rospy.spin()

if __name__ == '__main__':
    main()