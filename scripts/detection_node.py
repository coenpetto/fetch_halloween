#!/usr/bin/env python

import rospy
import cv2
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fetch_halloween.msg import HumanDetected
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from ArmController import ArmController

soundAssets = '/home/catkin_ws/src/fetch-halloween/sounds/'
sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
          'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']
throttle = 3  # seconds
moving = False


def play_sound():
    global allow_yak
    if rospy.Time.now() <= allow_yak:           # Throttles yak to avoid
        rospy.logwarn("Sound client throttled")      # SoundClient segfault
        return
    # when to reallow yak
    allow_yak = rospy.Time.now() + rospy.Duration.from_sec(throttle)
    soundhandle.playWave(soundAssets + random.choice(sounds))


def offer_candy():
    global arm
    global soundhandle

    # Extend the arm to offer the bowl
    arm.extend()

    # Say something
    play_sound()

    # Wait for some time
    rospy.sleep(5)

    # Tuck the arm back in
    arm.tuck()


def image_callback(img_msg):
    global pub
    global bridge

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        # Send a message so that the robot stops moving - CURRENTLY SENDS ON EVERY MESSAGE
        # TODO: ADD DETECTION CODE HERE

        if not moving:
            # One routine at a time
            moving = True

            # Send a message so the base stops moving
            detection_msg = HumanDetected()
            detection_msg.detected = True
            pub.publish(detection_msg)

            offer_candy()

            # Send a message so the base can move again
            detection_msg = HumanDetected()
            detection_msg.detected = False
            pub.publish(detection_msg)

            moving = False

    except CvBridgeError as e:
        print(e)
        return


if __name__ == '__main__':
    # Init the node
    rospy.init_node("detection_node")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Init the tools
    bridge = CvBridge()
    pub = rospy.Publisher('/human_detected', HumanDetected, queue_size=10)
    arm = ArmController()
    soundhandle = SoundClient()
    allow_yak = rospy.Time.now()

    # Get frames from the robot's camera
    image_topic = "/head_camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()
