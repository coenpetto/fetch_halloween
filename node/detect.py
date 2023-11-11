#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Face detection code using OpenCV
    # ...

    # Display the image with rectangles around detected faces
    cv2.imshow("Face Detection", cv_image)
    cv2.waitKey(3)

def face_detection_node():
    rospy.init_node('face_detection_node', anonymous=True)
    rospy.Subscriber("/your/image/topic", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        face_detection_node()
    except rospy.ROSInterruptException:
        pass
