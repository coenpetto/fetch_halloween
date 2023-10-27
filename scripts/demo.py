#!/usr/bin/env python

import rospy

from BaseController import BaseController
from fetch_halloween.msg import HumanDetected


def detection_callback(msg):
    global base
    rospy.loginfo("Human detected")

    if msg.detected:
        # stop moving the base
        base.cancel_movement()

    else:
        # resume moving the base
        base.goto(0, 0, 1.57)


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Need to keep track of which goal we're on
    target_goal = 1

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    base = BaseController()

    detection_topic = "/human_detection"
    rospy.Subscriber(detection_topic, HumanDetected, detection_callback)

    rospy.loginfo("Moving to goal 1")
    base.goto(0, 0, 1.57)
