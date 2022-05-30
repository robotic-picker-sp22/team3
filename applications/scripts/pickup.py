#! /usr/bin/env python3

from time import sleep
from gripper_marker import gripper_markers
from perception import SHELF_FRAME_NAME

import robot_api
import rospy
import copy
from geometry_msgs.msg import PoseStamped, Quaternion
from perception_msgs.msg import ObjectPose
from visualization_msgs.msg import Marker
from perception import SHELF_HEIGHT
import json

''' Run before starting
The AR marker stuff:
    roslaunch robot_api ar_desktop.launch cam_image_topic:=mock_point_cloud
Bin cropper:
    rosrun applications bin_marker_cropper.py
(SIM ONLY) cloud publisher:
    rosrun applications publish_saved_cloud.py scene2.bag
Point cloud demo:
    roslaunch perception point_cloud_demo.launch data_dir:=/home/capstone/catkin_ws/src/fetch-picker/combined_labels
(Run on robot IRL) MoveIt:
    roslaunch robot_api move_group.launch
Then run this code
'''

def print_usage():
    print('Picks up an object')
    print('Usage: rosrun applications pickup.py object_name')


TAG_POSE_TOPIC = '/ar_pose_marker'
OBJECT_POSE_TOPIC = "/object_pose"
PRE_PICKUP_DIST = 0.16
GRIPPER_LENGTH = 0.166
GRIPPER_MARGIN = 0.04
FINGER_LENGTH = 0.03


AR_FLAT = Quaternion()
# AR_FLAT.x = 0
# AR_FLAT.y = 0
# AR_FLAT.z = 0
# AR_FLAT.w = 1
AR_FLAT.x = 0.0
AR_FLAT.y = 0
AR_FLAT.z = .707
AR_FLAT.w = -.707


START_POSE = PoseStamped()
START_POSE.header.frame_id = "torso_lift_link"
START_POSE.pose.position.x = 0.45
START_POSE.pose.position.y = 0.3
START_POSE.pose.position.z = 0.2805710911750793
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

    # The list of visible objects
    objects = []

    # When true, stops recording object poses
    pause_perception = False

    # Current pose of the gripper/arm
    _gripper_pose: PoseStamped = None

    def __init__(self, db_fpath: str, listen=True, silent=False) -> None:
        rospy.loginfo('Initializing Pickup object...')
        self.silent = silent
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.obj_db = self._parse_db_file(db_fpath)
        self._init_robot()
        rospy.Subscriber(OBJECT_POSE_TOPIC, ObjectPose, self._get_obj_data)                                                 

    def _init_robot(self):
        ''' Initializes the robot arm, head, torso, and gripper
        '''
        rospy.loginfo('Init robot hardware...')
        self.torso = robot_api.Torso()
        self.arm = robot_api.Arm()
        self.head = robot_api.Head()
        self.gripper = robot_api.Gripper()
        self.torso.set_height(0.4)
        self.start_pose()
        self.head.look_at('base_link', 0.8, 0, SHELF_HEIGHT-0.08)
        rospy.loginfo("Robot hardware initialized.")

    def _get_obj_data(self, msg):
        ''' Callback from the object pose subscriber to update the local
        object names, poses, and dimensions for what's visible to the robot
        '''
        # old_objects = list(self.objects)
        old_names = [obj['name'] for obj in self.objects]
        self.objects.clear()
        if self.pause_perception:
            return
        for name, pose, dimensions in zip(msg.names, msg.poses, msg.dimensions):
            new_obj = {"name": name, "pose": pose, "dimensions": dimensions}
            if new_obj['name'] in old_names:
                old_names.remove(new_obj['name'])
            else:
                rospy.loginfo(f'Found new object {name}')
            self.objects.append(new_obj)
        for old_name in old_names:
            rospy.loginfo('Lost view of %s' % old_name)
        # if not self.silent:
        #     rospy.loginfo(f"Sees {len(self.objects)} objects")

    def get_object(self, obj_name: str) -> dict:
        ''' Returns the object descriptor dictionary, or None
        if couldn't find the object
        '''
        obj_name = obj_name.lower()
        for object in self.objects:
            if object['name'].lower() == obj_name:
                return object
        return None

    
    def old_pickup(self, name):
        ''' DEPRECIATED
        Pre-picks, picks, and drops the selected object
        '''
        object = self.get_object(name)
        if object is None:
            if not self.silent:
                rospy.logerr(f"Couldn't find object {name}")
            return False
        if not self.silent:
            rospy.loginfo(f"Picking up {name}")
        self.pause_perception = True
        result = True
        try:
            self.prepick(object)
            self.pick(object)
            self.place(object)
        except:
            result = False
        self.pause_perception = False
        return result


    def copy_pose(self, obj):
        ''' Returns a new copied pose from the object
        '''
        pose = PoseStamped()
        pose.header.frame_id = obj['pose'].header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = copy.deepcopy(obj['pose'].pose.position)
        # pose.pose.position.x = obj['pose'].pose.position.x
        # pose.pose.position.y = obj['pose'].pose.position.y
        # pose.pose.position.z = obj['pose'].pose.position.z
        pose.pose.orientation = AR_FLAT
        return pose

    '''
    MAIN PICKING FUNCTIONS
    The three functions make up the 3 pieces of the picking logic, where
    each part has a required starting and ending state

    PREPICK:
        START: the gripper must be in a neutral position outside of the 
               shelf before this function is called
        END: ends in a position directly in front of the selected object
             ready to move forward and pick
    PICK:
        START: starts in a position directly in front of the object
        END: must end grasping the object in a neutral location outside of 
             the shelf
    PLACE: 
        START: outside of the shelf grasping the object
        ENDS: in a neutral position outside of the shelf
    '''


    def prepick(self, obj):
        ''' Gets ready to pick up the object
        '''
        self.pause_perception = True
        rospy.loginfo(f'prepick {obj["name"]}')
        print(obj['pose'])
        self.gripper.close()
        if not self.clear_area(obj):
            pose = self.copy_pose(obj)
            # Get the gripper into the pre-pickup position
            pose.pose.position.z += GRIPPER_MARGIN
            pose.pose.position.y += GRIPPER_LENGTH + PRE_PICKUP_DIST
            if not self.move_to_pose(pose):
                rospy.logerr("Failed to move to pregrasp.")
                return False
        return True
        # self.pause_perception = False


    def pick(self, obj):
        ''' Moves the gripper forward to pick up the obj and
        out of the bin
        '''
        rospy.loginfo(f'pick {obj["name"]}')
        self.pause_perception = True
        print(f'object {obj["name"]}\n{obj["pose"]}')
        result = self.pickup_object(obj)
        self.pause_perception = False
        return result
        

    def place(self, obj):
        self.drop()
        self.start_pose()


    def pickup_object(self, obj):
        pose = self.copy_pose(obj)
        obj_depth = obj['dimensions'].y/2 + 0.01
        obj_width = obj['dimensions'].x + 0.01
        # Move to just infront of the object
        rospy.loginfo(f'obj depth {obj_depth}')
        pose.pose.position.y += GRIPPER_LENGTH + FINGER_LENGTH + obj_depth
        pose.pose.orientation = AR_FLAT
        if not self.move_to_pose(pose): 
            rospy.logerr('Just pre-grasp error')
            return False
        # Grasp the object
        rospy.loginfo(f"Opening gripper to width {obj_width}")
        self.gripper.open(obj_width)
        pose.pose.position.y -= FINGER_LENGTH + 0.03
        if not self.move_to_pose(pose): 
            rospy.logerr('Grasp error')
            return False
        effort = int(self.obj_db[obj['name']]['grip_effort'])
        rospy.loginfo(f'Using effort {effort} for {obj["name"]}.')
        self.gripper.close(max_effort=effort)
        # Move out of the container
        pose.pose.position.z += GRIPPER_MARGIN
        pose.pose.position.y += PRE_PICKUP_DIST
        if not self.move_to_pose(pose): return False
        return True

    def visualize_gripper(self, pose: PoseStamped):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.header.frame_id = pose.header.frame_id
        marker.pose = pose.pose
        # Scale
        marker.scale.x = GRIPPER_LENGTH
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        # Color
        marker.color.r = 1.0
        marker.color.a = 0.6
        self._marker_publisher.publish(marker)


    def drop(self):
        if not self.silent:
            rospy.loginfo("Starting Drop")
        self.move_to_pose(START_POSE)
        if not self.silent:
            rospy.loginfo("Moving to Drop 1")
        self.move_to_pose(DROP_1)
        if not self.silent:
            rospy.loginfo("Moving to Drop 2")
        self.move_to_pose(DROP_2)
        if not self.silent:
            rospy.loginfo("Moving to Drop 3")
        self.move_to_pose(DROP_3)
        self.gripper.open()
        if not self.silent:
            rospy.loginfo("Finished Drop")
    
    def start_pose(self):
        if not self.silent:
            rospy.loginfo("Going to start pose")
        for i in range(5):
            if self.move_to_pose(START_POSE):
                return
            sleep(1)
        rospy.logerr('Could not move to start')
        exit(-1)


    def move_to_pose(self, pose: PoseStamped) -> bool:
        self.visualize_gripper(pose)
        result = self.arm.move_to_pose_ik(pose)
        self._gripper_pose = pose
        return result


    def clear_area(self, object: dict):
        ''' Clears all objects away from the path into the bin for 
        the selected object
        Returns true if it moved objects, false otherwise
        '''
        BIN_CENTER = -0.14
        goal_obj_left = object['pose'].pose.position.x > BIN_CENTER
        objs_front = self.any_infront(object)
        if len(objs_front) < 1:
            # rospy.loginfo(f'No objects infront of {object["name"]}')
            return False
        for of in objs_front:
            self.pause_perception = True
            self.push_object(of, goal_obj_left)
            self.pause_perception = False
        return True


    def push_object(self, object: dict, to_right: bool):
        ''' Pushes object to either side of the bin. Pushes
        right if to_right set, pushes left otherwise.
        '''
        rospy.loginfo('Pushing %s out of the way' % object['name'])
        pose = self.copy_pose(object)
        dimensions = object['dimensions']
        pose.pose.orientation = AR_FLAT
        # Pre-push position
        offset = dimensions.x/2 + GRIPPER_MARGIN
        if not to_right: offset *= -1
        pose.pose.position.x += offset
        pose.pose.position.y += GRIPPER_LENGTH + PRE_PICKUP_DIST
        pose.pose.position.x += GRIPPER_MARGIN
        self.move_to_pose(pose)
        # Just abt to push position
        pose.pose.position.y -= PRE_PICKUP_DIST
        self.move_to_pose(pose)
        # Push position
        pose.pose.position.x = -0.10 #TODO: change this to be variable
        self.move_to_pose(pose)
        # Back off
        offset = GRIPPER_MARGIN 
        if not to_right: offset *= -1
        pose.pose.position.x += offset
        self.move_to_pose(pose)



    def any_infront(self, object: dict) -> list:
        ''' Checks if any other objects are infront of the provided 
        object, and returns a list of objects infront
        '''
        objects_infront = []
        for other_obj in self.objects:
            if other_obj == object: continue
            if self.infront(object, other_obj):
                rospy.loginfo('%s infront of %s' % (other_obj['name'], object['name']))
                objects_infront.append(other_obj)
        if len(objects_infront) == 0:
            rospy.loginfo('No objects infront of %s' % object['name'])
        return objects_infront


    def infront(self, obj_b: dict, obj_i: dict):
        ''' Returns true if object_behind is behind object_infront,
        false otherwise
        '''
        # Check object behind is further into the bin
        if obj_b['pose'].pose.position.y >= obj_i['pose'].pose.position.y:
            rospy.loginfo('%s toward front of bin of %s' % (obj_b['name'], obj_i['name']))
            return False
        # Else behind is further into bin, check horizontal overlap
        b_min = obj_b['pose'].pose.position.x - obj_b['dimensions'].x/2
        b_max = obj_b['pose'].pose.position.x + obj_b['dimensions'].x/2
        i_min = obj_i['pose'].pose.position.x - obj_i['dimensions'].x/2
        i_max = obj_i['pose'].pose.position.x + obj_i['dimensions'].x/2
        rospy.loginfo('obj_b %s' % obj_b['name'])
        rospy.loginfo(f'b_min {b_min}')
        rospy.loginfo(f'b_max {b_max}')
        rospy.loginfo('obj_i %s' % obj_i['name'])
        rospy.loginfo(f'i_min {i_min}')
        rospy.loginfo(f'i_max {i_max}')
        min_in_range = b_min < i_min and i_min < b_max
        max_in_range = b_min < i_max and i_max < b_max
        bigger = i_min < b_min and b_max < i_max
        rospy.loginfo(f'min_in_range {min_in_range}')
        rospy.loginfo(f'max_in_range {max_in_range}')
        rospy.loginfo(f'bigger {bigger}')
        return min_in_range or max_in_range or bigger

    
    def _parse_db_file(self, fpath: str):
        db = {}
        if fpath.endswith('.csv'):
            with open(fpath, 'r') as file:
                for line in file:
                    print(line)
                    objectName, row, col = line.split(",")
                    objectName = objectName.strip()
                    if objectName not in db:
                        db[objectName] = {'bins': []}
                    row = int(row)
                    col = int(col)
                    db[objectName]['bins'].append((row,col))
        elif fpath.endswith('.json'):
            with open(fpath, 'r') as file:
                db = dict(json.load(file))
        else:
            rospy.logerr(f'Unexpected file type {fpath}')
        return db


def main():
    rospy.init_node('pickup_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    pickup = Pickup('/home/capstone/catkin_ws/src/fetch-picker/object_db_files/db.json')
    sleep(1)
    rospy.loginfo(f"Finished initializing pickup")
    pickup.pickup(argv[1])
    # pickup.start_pose()
    # rospy.spin()

if __name__ == '__main__':
    main()