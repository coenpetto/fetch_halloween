import actionlib
import rospy
import tf
from math import sin, cos, radians
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BaseController(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

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

    def get_current_orientation(self, frame="base_link"):
        listener = tf.TransformListener()
        listener.waitForTransform(
            "/map", frame, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/map", frame, rospy.Time(0))
        return rot

    def rotate_left(self, angle_degrees, frame="base_link"):
        current_rot = self.get_current_orientation(frame)
        (roll, pitch, yaw) = euler_from_quaternion(current_rot)
        yaw += radians(angle_degrees)
        q = quaternion_from_euler(roll, pitch, yaw)
        self.goto(0, 0, yaw, frame)

    def rotate_right(self, angle_degrees, frame="base_link"):
        current_rot = self.get_current_orientation(frame)
        (roll, pitch, yaw) = euler_from_quaternion(current_rot)
        yaw -= radians(angle_degrees)
        q = quaternion_from_euler(roll, pitch, yaw)
        self.goto(0, 0, yaw, frame)
