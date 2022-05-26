#! /usr/bin/env python3
import rospy
import sys
import actionlib
import os

from picking_msgs.srv import PickupObjectList, PickupObjectListResponse
from picking_msgs.msg import *
from perception import crop_to_bin
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
        self._picker_client.wait_for_server()

    def _parse_db_file(self, fpath: str):
        db = {}
        with open(fpath, 'r') as file:
            for line in file:
                print(line)
                objectName, x, y = line.split(",")
                objectName = objectName.strip()
                x = int(x)
                y = int(y)
                db[objectName] = db.get(objectName, [])
                db[objectName].append((x,y))
        return db

    def _pick(self, objectName: str):
        locations = self.object_db.get(objectName, [])
        if len(locations) == 0:
            raise Exception()
        row, col = self.object_db[objectName].pop()
        crop_to_bin(row, col)
        goal = PickupRequestGoal()
        goal.name = objectName
        self._picker_client.send_goal(goal)
        self._picker_client.wait_for_result()
        return self._picker_client.get_result()
    
    def _pick_list(self, objectList):
        rospy.logerr(objectList)
        for item in objectList:
            self._pick(item)

    def handle_pick_list(self, req: PickupObjectList):
        rospy.logerr("handling pick list")
        self._pick_list(req.objects)
        return PickupObjectListResponse()

def main(db_fpath: str):
    rospy.init_node('main_picker_node')
    wait_for_time()

    server = MainPickerNode(db_fpath)
    service = rospy.Service('main_picker_object_list', PickupObjectList, server.handle_pick_list)
    rospy.logerr("main_picker_node is ready to accept requests")
    rospy.spin()




if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: main_picker_node.py path/to/database_file.csv")
        sys.argv.append('../object_db_files/test.csv')
    rospy.logerr(os.getcwd())
    rospy.logerr(sys.argv[1])
    main(sys.argv[1])