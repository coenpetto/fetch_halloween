import rospy
import cv2
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fetch_halloween.msg import HumanDetected
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from ArmController import ArmController

# sudo apt-get install ros-melodic-sound-play
# rosdep install sound_play


class DetectionController(object):

    def __init__(self):
        self.arm = ArmController()
        self.soundhandle = SoundClient()
        self.bridge = CvBridge()
        self.allow_yak = rospy.Time.now()
        self.throttle = 3  # seconds
        self.sound_assets = '/home/catkin_ws/src/fetch-halloween/sounds/'
        self.sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
                       'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']
        self.completed = False

    def begin_detection(self):
        # Get frames from the robot's camera
        image_topic = "/head_camera/rgb/image_raw"
        subscriber = rospy.Subscriber(image_topic, Image, self.image_callback)

        while not self.completed and not rospy.is_shutdown():
            rospy.sleep(0.1)  # Spin the class until the candy routine is over

        subscriber.unregister()

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            cv2.imshow("Image Window", cv_image)
            cv2.waitKey(3)

            # TODO: ADD DETECTION CODE HERE

            self.offer_candy()

        except CvBridgeError as e:
            print(e)
            return

    def offer_candy(self):
        # Extend the arm to offer the bowl
        self.arm.extend()

        # Say something
        self.play_sound()

        # Wait for some time
        rospy.sleep(5)

        # Tuck the arm back in
        self.arm.tuck()

    def play_sound(self):
        if rospy.Time.now() <= self.allow_yak:           # Throttles yak to avoid
            rospy.logwarn("Sound client throttled")      # SoundClient segfault
            return
        # when to reallow yak
        self.allow_yak = rospy.Time.now() + rospy.Duration.from_sec(self.throttle)
        self.soundhandle.playWave(
            self.sound_assets + random.choice(self.sounds))
