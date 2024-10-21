#!/usr/bin/env python

import rospy

from BaseController import BaseController
from DetectionController import DetectionController

goals = {
    "lobby": [2.27844619751, 2.7609167099, 0.358169555664],
    # "main door": [4.678, 4.616, 2.79253],
    "back door": [3.01748561859, 5.96243572235, -0.000566482543945],
    "side door": [4.58303928375, 5.43015241623, 0.00215721130371]
}

if __name__ == "__main__":
    # Create a node
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
