#!/usr/bin/env python

import rospy

from DetectionController import DetectionController

goals = [1, 2, 3, 4, 5]

if __name__ == "__main__":
    # Create a node
    rospy.init_node("detection_test")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    detection = DetectionController()

    for goal in goals:
        rospy.loginfo("Starting detection")
        detection.begin_detection()
        rospy.loginfo("Detection complete")
