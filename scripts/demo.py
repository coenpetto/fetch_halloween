#!/usr/bin/env python

import rospy

from ArmController import ArmController
from BaseController import BaseController
from fetch_halloween.msg import HumanDetected


# TODO: Detection node that publishes when an object is detected

def detection_callback():
    rospy.logingo("Human detected")
    # stop moving until the person is gone


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    base = BaseController()
    arm = ArmController()

    detection_topic = "/human_detection"
    rospy.Subscriber(detection_topic, HumanDetected, detection_callback)

    rospy.loginfo("Moving to goal 1")
    base.goto(0, 0, 1.57)
