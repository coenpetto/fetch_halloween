import rospy
import cv2
import random
import cv2
import imutils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from ArmController import ArmController
from BaseController import BaseController

# sudo apt-get install ros-melodic-sound-play
# rosdep install sound_play


class DetectionController(object):

    def __init__(self):
        self.arm = ArmController()
        self.base = BaseController()
        self.soundhandle = SoundClient()
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.allow_yak = rospy.Time.now()
        self.throttle = 3  # seconds
        self.sound_assets = '/home/bot_ws/src/fetch-halloween/sounds/'
        self.sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
                       'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']
        self.target_detection = {
            "area": 0,
            "image_width": 0,
            "dims": (0, 0, 0, 0)
        }
        self.completed = False

    def begin_detection(self):
        """ Start the detection routine """

        # Wait for the HOG to initialize
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        rospy.sleep(1)

        # Get frames from the robot's camera
        image_topic = "/head_camera/rgb/image_raw"
        subscriber = rospy.Subscriber(image_topic, Image, self.image_callback)

        # Start following the person
        follow_person_thread = rospy.Timer(
            rospy.Duration(0.1), self.follow_person)

        # Continue following for some seconds
        rospy.sleep(5)

        # Stop following the person
        follow_person_thread.shutdown()

        # Offer the candy
        self.offer_candy()

        while not self.completed and not rospy.is_shutdown():
            rospy.sleep(0.1)  # Spin the class until the candy routine is over

        subscriber.unregister()

    def follow_person(self, timer):
        """ Follow the person by rotating left or right depending on where they are in the frame"""

        x, y, w, h = self.target_detection["dims"]

        # If there is a detection
        if w != 0:
            # Is the detection to the left or right of the center of the image frame?
            if (x + (w / 2)) < (self.target_detection["image_width"] / 2):
                self.base.rotate_left(0.1)
            elif (x + (w / 2)) > (self.target_detection["image_width"] / 2):
                self.base.rotate_right(0.1)
        else:
            rospy.loginfo("No detection")

    def image_callback(self, img_msg):
        """ Process the image messages from ROS and store the closest detection in self.target_detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            image = imutils.resize(cv_image,
                                   width=min(400, cv_image.shape[1]))

            (regions, _) = self.hog.detectMultiScale(image,
                                                     winStride=(4, 4),
                                                     padding=(4, 4),
                                                     scale=1.05)

            for (x, y, w, h) in regions:
                # Draw rectangle around detection
                cv2.rectangle(image, (x, y),
                              (x + w, y + h),
                              (0, 0, 255), 2)

                # Calculate the area of each detection
                area = w * h
                # If it is bigger (closer) than the previous biggest, use it as the tracking target
                if area > self.target_detection["area"]:
                    self.target_detection["area"] = area
                    self.target_detection["image_width"] = image.shape[1]
                    self.target_detection["dims"] = (x, y, w, h)

            # Put the image on the computer screen
            cv2.imshow("Image Window", image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            print(e)
            return

    def offer_candy(self):
        """ Offer the candy to the person """
        # Extend the arm to offer the bowl
        # self.arm.extend()

        # Say something
        self.play_sound()

        # Wait for some time
        rospy.sleep(15)

        # Tuck the arm back in
        # self.arm.tuck()

        # Exit the entire sequence
        self.completed = True

    def play_sound(self):
        """ Play a random sound from the list """
        if rospy.Time.now() <= self.allow_yak:           # Throttles yak to avoid
            rospy.logwarn("Sound client throttled")      # SoundClient segfault
            return
        # when to reallow yak
        self.allow_yak = rospy.Time.now() + rospy.Duration.from_sec(self.throttle)
        self.soundhandle.playWave(
            self.sound_assets + random.choice(self.sounds))
