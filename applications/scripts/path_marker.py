#!/usr/bin/env python3
import rospy
import math
import nav_msgs.msg
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

class NavPath(object):
    def __init__(self):
        self._path = []
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        rospy.sleep(0.5)
            
    def publishingPath(self):
        marker = Marker(
                type=Marker.LINE_STRIP,
                # id=0,
                lifetime=rospy.Duration(10),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                points = self._path)
        self.marker_publisher.publish(marker)


    def callback(self, msg):
        if len(self._path)==0 or 0.5<self.distance(msg.pose.pose.position):
            self._path.append(msg.pose.pose.position)
            self.publishingPath()
    
    def distance(self, point):
        prev_point = self._path[-1]
        return math.dist(self.listMaking(prev_point), self.listMaking(point))

    def listMaking(self, p1):
        return [p1.x, p1.y, p1.z]

    





def main():
                                                               
    rospy.init_node('nice_node')
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
  main()