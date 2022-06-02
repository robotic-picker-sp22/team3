#! /usr/bin/env python3
import rospy
import sys
import actionlib
import os
import json
import robot_api

from picking_msgs.srv import PickupObjectList, PickupObjectListResponse
from picking_msgs.msg import PickRequestGoal, PickRequestAction
from perception import crop_to_bin, COL_POSITIONS, BIG_ROW_POSITIONS, SMALL_COLUMNS, SMALL_ROW_POSITIONS, SHELF_HEIGHT
from pickup_server import STATUS_FAILED, STATUS_SUCCEEDED


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MainPickerNode:

    def __init__(self, db_fpath: str):
        # read from file
        self.object_db = self._parse_db_file(db_fpath)
        self._picker_client = actionlib.SimpleActionClient('pick_request', PickRequestAction)
        rospy.loginfo("Waiting for picking server...")
        self._picker_client.wait_for_server()
        rospy.loginfo('Found picking server.')
        self._crop_row = None
        self._crop_col = None
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()

        
    def _crop(self, row: int, col: int):
        ''' Crops the vision point cloud to a given bin row and column
        '''
        crop_to_bin(row, col)
        # Wait if the new bin is different than previous
        if row != self._crop_row or col != self._crop_col:
            # Change height for the new row
            row_pos = SMALL_ROW_POSITIONS[row] if col in SMALL_COLUMNS else BIG_ROW_POSITIONS[row]
            row_height = SHELF_HEIGHT + row_pos
            torso_height = row_height - 0.3 + 0.18
            torso_height = max(torso_height, 0)
            torso_height = min(0.4, torso_height)
            self._torso.set_height(torso_height)
            # Look at the new row
            self._head.look_at('base_link', 0.5, 0, row_height)
            self._crop_row = row
            self._crop_col = col
            rospy.sleep(4)


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
    

    def _pick_feedback_cb(self, feedback):
        ''' Prints out feedback from the picking action
        '''
        rospy.loginfo(feedback.state)


    def _pick(self, objectName: str):
        ''' Picks up a given object, first crops to the expected bin for the object,
        then sends a pick goal through the pick action client.
        Returns: True if object successfully picked and placed into the box, 
        False otherwise
        '''
        locations = self.object_db.get(objectName, [])
        if objectName not in self.object_db:
            rospy.logerr(f"Object not in database {objectName}")
            rospy.logwarn(self.object_db)
            return False
        locations = self.object_db[objectName]['bins']
        if len(locations) == 0:
            rospy.logerr(f"No locations found for object {objectName}")
            return False
        rospy.loginfo(f'\n\nAttempting to pick {objectName}...\n\n')
        row, col = locations[-1]
        self._crop(row, col)

        goal = PickRequestGoal()
        goal.name = objectName
        self._picker_client.send_goal(goal, feedback_cb=self._pick_feedback_cb)
        self._picker_client.wait_for_result()
        
        result = self._picker_client.get_result()
        if result.status == STATUS_FAILED:
            rospy.logwarn(f'Could not pick up {objectName}')
            # Add the object back to the database
            # locations.append((row, col))
            return False
        elif result.status == STATUS_SUCCEEDED:
            rospy.loginfo(f"Successfully picked up {objectName}!")
        return True
    

    def _pick_list(self, objectList):
        ''' Pick up all objects in the list
        '''
        rospy.loginfo(f'Picking up objects {objectList}')
        for item in objectList:
            if not self._pick(item):
                rospy.logerr(f"Cannot find item. Skipping {item}")


    def handle_pick_list(self, req: PickupObjectList):
        rospy.loginfo("handling pick list")
        self._pick_list(req.objects)
        return PickupObjectListResponse()


def main(db_fpath: str):
    rospy.init_node('main_picker_node')
    wait_for_time()

    server = MainPickerNode(db_fpath)
    service = rospy.Service('main_picker_object_list', PickupObjectList, server.handle_pick_list)
    rospy.loginfo("main_picker_node is ready to accept requests")
    rospy.spin()



DEFAULT_DB_FILE = '/home/capstone/catkin_ws/src/fetch-picker/object_db_files/db.json'
# DEFAULT_DB_FILE = '../object_db_files/test.csv'

def print_usage():
    print("Usage: main_picker_node.py path/to/database_file.csv")

if __name__ == "__main__":
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        print(f"Using default file: {DEFAULT_DB_FILE}")
        db_file = DEFAULT_DB_FILE
    elif len(argv) > 2:
        print_usage()
        print('Too many arguments')
        exit()
    else:
        db_file = os.path.abspath(argv[1])
        if not os.path.isfile(db_file):
            rospy.logerr(f"{db_file} does not exist")
            exit()
    print(f'Using database path: {db_file}')
    main(db_file)