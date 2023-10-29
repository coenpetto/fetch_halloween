import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface


class ArmController(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
                       "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    def tuck(self):
        pose = [0.0, 1.5, 0.0, -2.3, 0.0, 1.7, 1.57]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(
                self.joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def extend(self):
        pose = [0.0, -0.5, 0.0, 0.8, 0.0, 0.9, 1.57]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(
                self.joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
