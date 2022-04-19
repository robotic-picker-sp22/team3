POSE_TOPIC = '/amcl_pose'
GOAL_TOPIC = '/move_base_simple/goal'

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class NavGoal(object):
    '''
    Sends navigation goals to the robot
    '''
    _latest_pose: PoseWithCovarianceStamped = None
    _locations = {}

    def __init__(self):
        # Create a subscriber
        self._pose_subscriber = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, callback=self._pose_callback)
        self._goal_publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)

    def save_current_pose(self, name: str):
        '''
        Save the most recent pose of the robot as a new location with name 
        '''
        msg = PoseStamped()
        msg.header = self._latest_pose.header
        msg.pose = self._latest_pose.pose.pose
        self._locations[name] = msg

    def get_locations(self):
        '''
        Return a list of locations the user has created
        '''
        return list(self._locations.keys())

    def goto(self, name: str) -> bool:
        '''
        Go to the location of name N, returns false if cannot find name
        '''
        if name in self._locations:
            self._goal_publisher.publish(self._locations[name])
            return True
        else:
            return False


    def delete(self, name) -> bool:
        '''
        Deletes the location with name N, returns false if cannot find name
        '''
        if name not in self._locations:
            return False
        self._locations.pop(name)
        return True

    def _pose_callback(self, msg):
        '''
        Called whenever the robot's pose is updated, records the most recent
        pose
        '''
        self._latest_pose = msg