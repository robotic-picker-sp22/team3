#!/usr/bin/env python3

from pickle import TRUE
import rospy
from pickup import Pickup


"""
loops:
  what item to pick
  what bin
"""

PICK = 'pick'
BIN = 'bin'
HELP = 'h'
QUIT = 'quit'

def pick_item(pickup: Pickup, name):
    pickup.pickup(name)

def change_bin(col, row):
    # rospy.set_param("bin_marker_col", col)
    # rospy.set_param("bin_marker_row", row)
    # print(f"bin changed to ({col}, {row})")
    print("this command doesn't do anything right now")

def print_help():
    print("Available commands:")
    print(f"  {PICK} [item_name]")
    print("    picks item if available in current bin")
    print(f"  {BIN} [col] [row]")
    print("    changes current bin to given location <NOT FUNCTIONAL>")
    print(f"  {HELP}")
    print("    prints available commands")
    print(f"  {QUIT}")
    print("    shuts down and exits the cli")

def main():
    print("Starting the pickup cli")
    print("  make sure roscore, moveit, perception_demo, ar tracker, and bin_marker_cropper are running")
    print()

    rospy.init_node("pickup_cli")
    while rospy.Time().now().to_sec() == 0:
        pass

    pickup = Pickup(silent=True)
    while(True):
        command = input("Enter command (h for help): ").split()
        
        if command[0] == PICK:
            pick_item(pickup, command[1])
        elif command[0] == BIN:
            change_bin(int(command[1]), int(command[2]))
        elif command[0] == HELP:
            print_help()
        elif command[0] == QUIT:
            break
        else:
            print(f"Unrecognized command: {command[0]}")
            print_help()
            
    print("shutting down...")

if __name__ == '__main__':
    main()