#!/usr/bin/env python

import rospy
from BaseController import BaseController

if __name__ == '__main__':
    rospy.init_node("test_nav")

    rospy.sleep(1)

    base = BaseController()
    base.rotate_left(45)
