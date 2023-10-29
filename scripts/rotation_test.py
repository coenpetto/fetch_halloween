#!/usr/bin/env python

import rospy
from BaseController import BaseController

if __name__ == '__main__':
    rospy.init_node("test_nav")

    rospy.sleep(1)

    base = BaseController()
    base.rotate_right(1.57)
    base.rotate_left(1.57)
