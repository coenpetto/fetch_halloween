#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fetch_halloween.msg import HumanDetected


def image_callback(img_msg):
    global pub
    global bridge

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        # Send a message so that the robot stops moving - CURRENTLY SENDS ON EVERY MESSAGE
        # ADD DETECTION CODE HERE
        detection_msg = HumanDetected()
        detection_msg.detected = True
        pub.publish(detection_msg)

        # TODO: Candy routine
    except CvBridgeError as e:
        print(e)
        return


if __name__ == '__main__':
    rospy.init_node("detection_node")

    bridge = CvBridge()
    pub = rospy.Publisher('/human_detected', HumanDetected, queue_size=10)

    # Get frames from the robot's camera
    image_topic = "/head_camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()
