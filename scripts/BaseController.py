import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos, radians


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

    def rotate_left(self, angle_degrees, frame="base_link"):
        current_theta = radians(angle_degrees)
        self.goto(0, 0, current_theta, frame)

    def rotate_right(self, angle_degrees, frame="base_link"):
        current_theta = -radians(angle_degrees)
        self.goto(0, 0, current_theta, frame)
