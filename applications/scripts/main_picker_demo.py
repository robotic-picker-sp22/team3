#! /usr/bin/env python3

import rospy
import sys, os
from picking_msgs.srv import PickupObjectList


def print_usage():
    print("Usage: main_picker_demo.py obj1 obj2 obj3")
    print("\nUsed to send a list of objects to the main picker node")

PICKER_SERVICE = 'main_picker_object_list'
def main(objects: list):
    rospy.wait_for_service(PICKER_SERVICE)
    print(f'DEMO SENDING OBJECT LIST:')
    print(objects)
    pickup_service = rospy.ServiceProxy(PICKER_SERVICE, PickupObjectList)
    pickup_service(objects)
    


if __name__ == "__main__":
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        exit()
    objects = [obj.strip().lower() for obj in argv[1:]]
    main(objects)