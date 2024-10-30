#!/usr/bin/env python

import rospy

from BaseController import BaseController
from DetectionController4 import DetectionController4

goals = {
    "lobby": [1.49356460571, 2.37614440918, -0.000372886657715],
    # "main door": [4.678, 4.616, 2.79253],
    "back door": [3.02198314667, 5.3306350708, 0.0060510635376],
    "side door": [4.31612586975, 5.35473537445, 0.00382804870605]
}

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    base = BaseController()
    detection = DetectionController4()

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
