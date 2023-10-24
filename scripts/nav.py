#!/usr/bin/env python

import rospy
from BaseController import BaseController

if __name__ == '__main__':
    rospy.init_node("test_extension")

    rospy.sleep(1)

    move_base = BaseController()
    move_base.goto(2.250, 3.118, 0.0)
