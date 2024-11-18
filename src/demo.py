#!/usr/bin/env python

import rospy

from BaseController import BaseController
from DetectionController import DetectionController

goals = {
    "lobby": [10.224, -3.066, 0.0],
    # "main door": [4.678, 4.616, 2.79253],
    "back door": [0.591, -2.182, 0.0],
    "side door": [-0.770, 3.259, 0.0]
}

if __name__ == "__main__":
    # Create a node
    # print(type(rospy.get_master().target))
    print(rospy.get_master().target)
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    base = BaseController()
    detection = DetectionController()

    # Move to the goals
    while True:
        for goal in goals:
            rospy.loginfo("Moving to %s...", goal)
            success = base.goto(goals[goal][0], goals[goal][1], goals[goal][2])

            # Continue to the next goal if the robot gets stuck
            if not success:
                rospy.loginfo("Failed to reach %s", goal)
                continue

            rospy.loginfo("Starting detection")
            detection.begin_detection()
