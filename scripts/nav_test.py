#!/usr/bin/env python

import rospy
from BaseController import BaseController

if __name__ == '__main__':
    rospy.init_node("test_nav")

    rospy.sleep(1)

    base = BaseController()
    base.goto(4.678, 4.616, 1.57)
