#! /usr/bin/env python3

import rospy
import actionlib
from picking_msgs.msg import *
from pickup import Pickup
from perception_msgs.msg import ObjectPose

OBJECT_POSE_TOPIC = "/object_pose"
STATUS_FAILED = "failed"
STATUS_SUCCEEDED = "success"

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class PickupServer():
    def __init__(self) -> None:
        self._pick_request_server = actionlib.SimpleActionServer('pick_request', PickRequestAction, execute_cb=self.execute_pick_request, auto_start=False)
        self._pick_request_server.start()
        self._pickup = Pickup()

    def execute_pick_request(self, goal: PickRequestGoal):
        feedback = PickRequestFeedback()

        for i in range(4):
            obj = self._pickup.get_object(goal.name)
            if obj is not None: break
            rospy.sleep(1)  # Wait 1 sec before trying again
        # rospy.logwarn(obj['pose'])
        if obj is None:
            result = PickRequestResult()
            result.status = STATUS_FAILED
            feedback.state = "Cannot find object"
            self._pick_request_server.publish_feedback(feedback)
            self._pick_request_server.set_aborted(result)
            return
        
        if self._pick_request_server.is_preempt_requested():
            result = PickRequestResult()
            result.status = "preempted"
            self._pick_request_server.set_preempted(result)
            return

        feedback.state = "pre-picking"
        self._pick_request_server.publish_feedback(feedback)
        self._pickup.prepick(obj)
        if self._pick_request_server.is_preempt_requested():
            result = PickRequestResult()
            result.status = "preempted"
            self._pick_request_server.set_preempted(result)
            return

        feedback.state = "picking"
        self._pick_request_server.publish_feedback(feedback)
        if not self._pickup.pick(obj):
            result = PickRequestResult()
            result.status = STATUS_FAILED
            feedback.state = "Cannot pickup object"
            self._pick_request_server.publish_feedback(feedback)
            self._pick_request_server.set_aborted(result)
            return
        if self._pick_request_server.is_preempt_requested():
            result = PickRequestResult()
            result.status = "preempted"
            self._pick_request_server.set_preempted(result)
            return

        feedback.state = "dropping"
        self._pick_request_server.publish_feedback(feedback)
        self._pickup.place(obj)

        feedback.state = "done"
        self._pick_request_server.publish_feedback(feedback)
        result = PickRequestResult()
        result.status = STATUS_SUCCEEDED
        self._pick_request_server.set_succeeded(result)


def main():
    rospy.init_node('pickup_server')
    wait_for_time()
    server = PickupServer()
    rospy.spin()


if __name__ == '__main__':
    main()
