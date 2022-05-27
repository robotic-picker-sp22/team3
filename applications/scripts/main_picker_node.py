#! /usr/bin/env python3
import rospy
import sys
import actionlib
import os

from picking_msgs.srv import PickupObjectList, PickupObjectListResponse
from picking_msgs.msg import PickRequestGoal, PickRequestAction
from perception import crop_to_bin
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
        self._picker_client.wait_for_server()
        

    def _parse_db_file(self, fpath: str):
        db = {}
        with open(fpath, 'r') as file:
            for line in file:
                print(line)
                objectName, row, col = line.split(",")
                objectName = objectName.strip()
                row = int(row)
                col = int(col)
                db[objectName] = db.get(objectName, [])
                db[objectName].append((row,col))
        return db
    

    def _pick_feedback_cb(self, feedback):
        rospy.loginfo(feedback.state)


    def _pick(self, objectName: str):
        locations = self.object_db.get(objectName, [])
        if len(locations) == 0:
            rospy.logerr(f"No bin found for {objectName}")
            rospy.logwarn(self.object_db)
            raise Exception()
        row, col = self.object_db[objectName].pop()
        crop_to_bin(row, col)
        rospy.sleep(2) # Wait for crop
        goal = PickRequestGoal()
        goal.name = objectName
        rospy.loginfo(f'Attempting to pick {objectName}...')
        self._picker_client.send_goal(goal, feedback_cb=self._pick_feedback_cb)
        self._picker_client.wait_for_result()
        # TODO: On failure add back object to db
        result = self._picker_client.get_result()
        if result.status == STATUS_FAILED:
            rospy.logwarn(f'Could not pick up {objectName}')
            self.object_db[objectName].append((row, col))
        elif result.status == STATUS_SUCCEEDED:
            rospy.loginfo(f"Successfully picked up {objectName}!")
        return result
    
    def _pick_list(self, objectList):
        rospy.loginfo(f'Picking up objects {objectList}')
        for item in objectList:
            try:
                self._pick(item)
            except:
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




if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: main_picker_node.py path/to/database_file.csv")
        sys.argv.append('../object_db_files/test.csv')
    rospy.loginfo(os.getcwd())
    rospy.loginfo(sys.argv[1])
    main(sys.argv[1])