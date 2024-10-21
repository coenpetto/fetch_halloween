#!/usr/bin/env python

import rospy

from BaseController import BaseController
from DetectionController import DetectionController

goals = {
    "lobby": [6.071, -2.995, 2.79253],
    # "main door": [4.678, 4.616, 2.79253],
    "back door": [-0.560, -18.986, -0.349066],
    "side door": [9.144, -7.081, 3.49066]
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
