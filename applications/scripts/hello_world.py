#! /usr/bin/env python3

import rospy


def main():
    rospy.init_node('hello_world')
    rospy.loginfo('Hello world!')


if __name__ == '__main__':
    main()
