import actionlib
import rospy
from math import sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus


class BaseController(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        self.rotation_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)

    def goto(self, x, y, theta, frame="map"):
        " Send a goal to the navigation stack "
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # wait for things to work
        self.client.send_goal(move_goal)
        res = self.client.wait_for_result()

        if not res:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False
        else:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                return True
            else:
                rospy.loginfo("Goal failed!")
                return False

    def rotate_right(self, duration):
        " Send Twist commands to rotate the robot to the right"
        rotate_cmd = Twist()
        rotate_cmd.angular.z = -0.5  # rad/s

        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.rotation_publisher.publish(rotate_cmd)
            rospy.sleep(0.1)

    def rotate_left(self, duration):
        " Send Twist commands to rotate the robot to the left"
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.5  # rad/s

        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.rotation_publisher.publish(rotate_cmd)
            rospy.sleep(0.1)

    def cancel_goals(self):
        " Cancel all goals in the navigation stack"
        self.client.cancel_all_goals()
