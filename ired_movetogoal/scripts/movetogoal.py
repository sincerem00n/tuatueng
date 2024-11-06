#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
import math


def pose_callback(pose_with_covariance):
    # print(pose_with_covariance)
    pose = pose_with_covariance.pose.pose
    print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))


def move_base_status_callback(status):
    pass


def move_base_result_callback(result):
    pass

def get_current_pose():
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    try:
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))

        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Could not get current pose: %s", e)
        return None

class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, theta):
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    def moveToPoint(self, x, y, theta):
        target_point = self.createGoal(x, y, theta)
        self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        print ("Move to %f, %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.z
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False
    def rotate(self, angle):
        angle_rad = math.radians(angle) if isinstance(angle, (int, float)) else angle
        quat = tf.transformations.quaternion_from_euler(0, 0, angle_rad)  # Fixed orientation
        goal_angle = MoveBaseGoal()
        
        current_pose = get_current_pose()
        if current_pose is None:
            rospy.logwarn("Could not get current pose")
            return None
        
        goal_angle.target_pose.header.frame_id = 'map'
        goal_angle.target_pose.header.stamp = rospy.Time.now()
        goal_angle.target_pose.pose = Pose(Point(current_pose.position.x, current_pose.position.y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        print("rotate sent")
        self.move_base_action.send_goal(goal_angle)
        return goal_angle


# Main program
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # TODO
    mba = moveBaseAction()
    while not rospy.is_shutdown():
        # mba.moveToPoint(1,0,1)
        mba.rotate(45)
        rospy.sleep(1)

        # mba.moveToPoint(0, 0, 0)
        # rospy.sleep(1)

    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass