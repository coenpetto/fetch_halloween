import rospy
import cv2
import random
import imutils
from playsound import playsound
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from ArmController import ArmController
from BaseController import BaseController
import math
import tf
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

class HogDetection(object):

    def __init__(self):
        self.arm = ArmController()
        self.base = BaseController()
        self.soundhandle = SoundClient()
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.allow_yak = rospy.Time.now()
        self.throttle = 3  # seconds
        self.sound_assets = '/home/tyler/catkin_ws/src/fetch_halloween/sounds/'
        self.sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
                       'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']
        self.completed = False
        self.current_yaw = None
        self.detections = []

        # Variables for angle tracking
        self.total_angle_traveled = 0.0
        self.prev_yaw = None

        # Subscribe to odometry to get current yaw
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize head action client
        self.head_client = actionlib.SimpleActionClient(
            'head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for head_controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Connected to head_controller action server.")

    def reset_head_position(self):
        """Reset the head position to look straight ahead."""
        # Create a goal to send to the head controller
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Pan and Tilt angles in radians
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
        rospy.loginfo("Head reset to straight ahead.")

    def odom_callback(self, msg):
        """Update the current yaw angle from odometry data and accumulate angle traveled."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)
        self.current_yaw = yaw

        # Accumulate the total angle traveled
        if self.prev_yaw is not None:
            delta_yaw = self.angle_difference(yaw, self.prev_yaw)
            self.total_angle_traveled += abs(delta_yaw)
        self.prev_yaw = yaw

    def begin_detection(self):
        """Start the detection routine with a 360-degree scan."""
        # Wait for the HOG to initialize
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        rospy.sleep(1)

        # Wait until current_yaw is available
        while self.current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Reset the head position
        self.reset_head_position()

        # Get frames from the robot's camera
        image_topic = "/head_camera/rgb/image_raw"
        subscriber = rospy.Subscriber(image_topic, Image, self.image_callback)

        # Start collecting detections
        self.detections = []

        # Reset angle tracking variables
        self.total_angle_traveled = 0.0
        self.prev_yaw = self.current_yaw

        # Keep rotating until we've rotated 360 degrees
        rotation_increment = 0.2  # Duration for each rotation increment (seconds)
        while not rospy.is_shutdown() and self.total_angle_traveled < 2 * math.pi - 0.1:
            # Rotate left for a small duration
            self.base.rotate_left(rotation_increment)
            rospy.sleep(rotation_increment)

        # Select the detection with the highest peopleness score
        if self.detections:
            best_detection = max(self.detections, key=lambda d: d['score'])
            # Rotate to face the person
            self.rotate_to_angle(best_detection['yaw'])
            # Offer the candy
            rospy.sleep(2)
            self.offer_candy()
        else:
            rospy.loginfo("No detections found.")

        # Cleanup
        subscriber.unregister()
        rospy.loginfo(
            "Finished offering candy: Exiting Detection Controller")

    def angle_difference(self, angle1, angle2):
        """Compute the smallest difference between two angles."""
        diff = angle1 - angle2
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return diff

    def rotate_to_angle(self, target_yaw):
        """Rotate the robot to the target yaw angle."""
        while self.current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rotate_duration_per_iteration = 0.1  # Duration for each rotation increment

        if self.angle_difference(target_yaw, self.current_yaw) > 0:
            rotate_method = self.base.rotate_left
        else:
            rotate_method = self.base.rotate_right

        # Rotate in small increments until the desired angle is reached
        while not rospy.is_shutdown():
            current_yaw = self.current_yaw
            angle_left = self.angle_difference(target_yaw, current_yaw)
            if abs(angle_left) < 0.05:  # Threshold for stopping
                break

            # Rotate for a small duration
            rotate_method(rotate_duration_per_iteration)
            rospy.sleep(rotate_duration_per_iteration)

        rospy.loginfo("Rotation to target angle complete.")

    def image_callback(self, img_msg):
        """Process the image messages from ROS and store detections with their scores and yaw."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            image = imutils.resize(cv_image,
                                   width=min(400, cv_image.shape[1]))

            (regions, weights) = self.hog.detectMultiScale(
                image,
                winStride=(4, 4),
                padding=(4, 4),
                scale=1.05)
            
            rospy.loginfo("regions len {}, weights len {}".format(len(regions), len(weights)))

            for ((x, y, w, h), weight) in zip(regions, weights):
                # Draw rectangle around detection
                cv2.rectangle(image, (x, y),
                              (x + w, y + h),
                              (0, 0, 255), 2)

                # Get the current yaw
                current_yaw = self.current_yaw

                # Store the detection
                detection = {
                    'area': w * h,
                    'score': weight,  # weight is a single float
                    'dims': (x, y, w, h),
                    'yaw': current_yaw,
                }
                self.detections.append(detection)

            # Display the image
            cv2.imshow("Image Window", image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            print(e)
            return

    def offer_candy(self):
        """Offer the candy to the person."""
        # Extend the arm to offer the bowl
        self.arm.extend()

        # Say something
        self.play_sound("voice")

        # Wait for some time
        rospy.sleep(10)

        # Warn about arm
        self.play_sound("warning")
        rospy.sleep(5)

        # Tuck the arm back in
        self.arm.tuck()

        # Exit the entire sequence
        self.completed = True

    def play_sound(self, cmd):
        """Play a random sound from the list."""
        if cmd == "voice":
            playsound(self.sound_assets + random.choice(self.sounds))
        else:
            playsound(self.sound_assets + "warning.wav")
