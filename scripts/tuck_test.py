#!/usr/bin/env python

import rospy
from ArmController import ArmController

if __name__ == '__main__':
    rospy.init_node("test_tuck")

    rospy.sleep(1)

    arm = ArmController()
    arm.tuck()
