import actionlib
import rospy
import tf
from math import sin, cos, radians
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans


class BaseController(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        self.current_pose = None
        self.odom_subscriber = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback)

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    def odom_callback(self, data):
        self.current_pose = data.pose.pose

    def rotate(self, theta, frame="base_link"):

        euler_angles = tf_trans.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        current_theta = euler_angles[2]

        new_theta = theta + current_theta

        self.goto(0, 0, new_theta, frame)
