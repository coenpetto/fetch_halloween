import rospy
import cv2
import random
import imutils
from playsound import playsound
from sensor_msgs.msg import Image, CameraInfo
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
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class YoloWith360(object):

    def __init__(self):
        self.arm = ArmController()
        self.base = BaseController()
        self.soundhandle = SoundClient()
        self.bridge = CvBridge()
        self.allow_yak = rospy.Time.now()
        self.throttle = 3  # seconds
        self.sound_assets = '/home/tyler/catkin_ws/src/fetch_halloween/sounds/'
        self.sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
                       'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']
        self.completed = False
        self.current_yaw = None
        self.detections = []
        self.person_detected = False  # Flag to indicate person detection

        # Image parameters
        self.image_width = None
        self.fov_horizontal = None  # Field of view in radians

        # Subscribe to odometry to get current yaw
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscribe to camera info to get image width and FOV
        rospy.Subscriber('/head_camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize head action client
        self.head_client = actionlib.SimpleActionClient(
            'head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for head_controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Connected to head_controller action server.")

    def camera_info_callback(self, msg):
        """Get image width and calculate horizontal FOV from camera info."""
        self.image_width = msg.width
        # Calculate horizontal FOV using camera intrinsic parameters
        fx = msg.K[0]  # Focal length in pixels along x-axis
        self.fov_horizontal = 2 * math.atan(self.image_width / (2 * fx))
        # rospy.loginfo("Image width: {}, FOV horizontal: {:.4f} radians".format(self.image_width, self.fov_horizontal))
        # Unsubscribe after obtaining the necessary info
        rospy.Subscriber('/head_camera/rgb/camera_info', CameraInfo, self.camera_info_callback).unregister()

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
        """Update the current yaw angle."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)
        self.current_yaw = yaw

    def begin_detection(self):
        """Rotate 360 degrees and lock in on the closest person detected."""
        rospy.sleep(1)

        # Wait until current_yaw is available
        while self.current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Wait until image width and FOV are available
        while (self.image_width is None or self.fov_horizontal is None) and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Reset the head position
        self.reset_head_position()

        # Subscribe to darknet_ros bounding boxes
        detection_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)

        # Initialize detection flag and detections list
        self.detections = []
        self.person_detected = False

        # Rotate 360 degrees
        rotation_increment = 0.1  # Duration for each rotation increment (seconds)
        target_rotation = 2 * math.pi  # 360 degrees in radians
        rotated_angle = 0.0
        previous_yaw = self.current_yaw

        rospy.loginfo("Starting 360-degree rotation for detection.")

        while not rospy.is_shutdown() and rotated_angle < target_rotation:
            # Rotate left for a small duration
            self.base.rotate_left(rotation_increment)
            rospy.sleep(rotation_increment)

            # Update rotated angle
            if self.current_yaw is not None:
                delta_yaw = self.angle_difference(self.current_yaw, previous_yaw)
                rotated_angle += abs(delta_yaw)
                previous_yaw = self.current_yaw
            else:
                rospy.logwarn("Current yaw is None during rotation.")

        # Unsubscribe from detections
        detection_subscriber.unregister()

        if self.detections:
            # Find the closest person based on the largest bounding box area
            closest_detection = max(self.detections, key=lambda d: d['area'])
            rospy.loginfo("Closest person detected with area: {:.2f}".format(closest_detection['area']))

            # Rotate to face the closest person
            self.rotate_to_angle(closest_detection['yaw'])

            # Offer the candy
            rospy.sleep(2)
            self.offer_candy()
        else:
            rospy.loginfo("No person detected after 360-degree rotation.")

        rospy.loginfo("Finished offering candy: Exiting Detection Controller")


    def angle_difference(self, angle1, angle2):
        """Compute the smallest difference between two angles."""
        diff = angle1 - angle2
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return diff

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def rotate_to_angle(self, target_yaw):
        """Rotate the robot to the target yaw angle."""
        while self.current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rotate_duration_per_iteration = 0.025  # Duration for each rotation increment

        while not rospy.is_shutdown():
            current_yaw = self.current_yaw
            angle_left = self.angle_difference(target_yaw, current_yaw)
            if abs(angle_left) < 0.005:  # Decrease threshold for higher accuracy
                break

            if angle_left > 0:
                rotate_method = self.base.rotate_left
            else:
                rotate_method = self.base.rotate_right

            # Rotate for a small duration
            rotate_method(rotate_duration_per_iteration)
            rospy.sleep(rotate_duration_per_iteration)

        rospy.loginfo("Rotation to target angle complete.")


    def detection_callback(self, msg):
        """Process detection results from darknet_ros and store person detections."""
        if self.image_width is None or self.fov_horizontal is None:
            rospy.logwarn("Image parameters not set yet.")
            return  # Skip processing until image parameters are available

        for bounding_box in msg.bounding_boxes:
            if bounding_box.Class == "person" and bounding_box.probability > 0.30:
                # Calculate the center of the bounding box
                bbox_center_x = (bounding_box.xmin + bounding_box.xmax) / 2.0

                # Reverse the sign of the angular offset
                angle_offset = -((bbox_center_x - self.image_width / 2.0) / self.image_width) * self.fov_horizontal

                # Adjust the target yaw
                target_yaw = self.normalize_angle(self.current_yaw + angle_offset)

                # Pan angle for head movement
                pan_angle = angle_offset  # Use angle_offset directly

                detection = {
                    'area': (bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin),
                    'score': bounding_box.probability,
                    'dims': (bounding_box.xmin, bounding_box.ymin,
                             bounding_box.xmax - bounding_box.xmin, bounding_box.ymax - bounding_box.ymin),
                    'yaw': target_yaw,      # Adjusted target yaw
                    'pan_angle': pan_angle  # Head pan angle to center person
                }
                self.detections.append(detection)
                rospy.loginfo("Person detected with score: {:.2f}, area: {:.2f}".format(bounding_box.probability, detection['area']))
                # rospy.loginfo("Angle offset: {:.4f} radians, Target yaw: {:.4f} radians".format(angle_offset, target_yaw))

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
