#!/usr/bin/env python

import rospy

from BaseController import BaseController
from DetectionController import DetectionController


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    base = BaseController()
    detection = DetectionController()

    rospy.loginfo("Moving to lobby...")
    base.goto(6.071, -2.995, 2.79253)

    detection.begin_detection()

    rospy.loginfo("Moving to main door...")
    base.goto(4.678, 4.616, 2.79253)

    detection.begin_detection()

    rospy.loginfo("Moving to back door...")
    base.goto(-0.560, -18.986, -0.349066)

    detection.begin_detection()

    rospy.loginfo("Moving to side door...")
    base.goto(9.144, -7.081, 3.49066)

    detection.begin_detection()
