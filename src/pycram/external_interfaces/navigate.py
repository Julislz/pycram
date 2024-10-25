<<<<<<< HEAD
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal


def queryPoseNav(navpose):
    """
    Sends a goal to the move_base Service. Connection to Giskard interface,
    will move robot to the given pose: navpose
    :param navpose: PoseStamped pose robot will navigate to
    """

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Move base")

    def done_callback(state, result):
        rospy.loginfo("Finished moving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    goal_msg = MoveBaseGoal()

    goal_msg.target_pose = navpose
    rospy.loginfo("navigating")
    client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result
=======
import math

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

from pycram.fluent import Fluent


import pycram.external_interfaces.giskard_new as giskardpy


class PoseNavigator():
    def __init__(self):
        rospy.loginfo("move_base init")
        global move_client
        self.client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        rospy.loginfo("Waiting for move_base ActionServer")
        if self.client.wait_for_server():
            rospy.loginfo("Done")
        #self.pub = rospy.Publisher('goal', PoseStamped, queue_size=10, latch=True)
        self.toya_pose = None
        self.goal_pose = None
        self.toya_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=100)

        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
        rospy.loginfo("move_base init construct done")

    def pub_fake_pose(self, fake_pose: PoseStamped):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position = fake_pose.pose.position
        msg.pose.pose.orientation = fake_pose.pose.orientation
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.toya_pose_pub.publish(msg)

    def toya_pose_cb(self, msg):
        self.toya_pose = msg.pose.pose.position
        rospy.sleep(0.1)

    def interrupt(self):
        print("interrupting hehe")
        self.client.cancel_all_goals()

    def pub_now(self, navpose: PoseStamped, interrupt_bool: bool = True) -> bool:
        self.goal_pose = navpose
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 0
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = navpose.pose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            return False

        rospy.loginfo(f"Publishing navigation pose")
        rospy.loginfo("Waiting for subscribers to connect...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            near_goal = False
            rospy.loginfo("Pose was published")
            if self.toya_pose is not None:
                dis = math.sqrt((self.goal_pose.pose.position.x - self.toya_pose.x) ** 2 +
                                (self.goal_pose.pose.position.y - self.toya_pose.y) ** 2)
                rospy.loginfo("Distance to goal: " + str(dis))
                if dis < 0.04 and interrupt_bool:
                    rospy.logwarn("Near Pose")
                    self.interrupt()
                    return True
                else:
                    self.client.wait_for_result()
                    rospy.logerr("robot needs more time")
                    return True
            else:
                rospy.logerr("something is wrong with navigation")
                return False


>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
