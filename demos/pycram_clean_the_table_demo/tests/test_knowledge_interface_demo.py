import time
from enum import Enum

import rospy.core

from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

from pycram.external_interfaces.knowrob import get_table_pose, get_location_pose, get_handle_pos, \
    init_knowrob_interface

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Fork", "Metalbowl", "Metalplate", "Spoon", "Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)


# x pose of the end of the couch table
table_pose = 4.84
# 5.2

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])
# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

handle_name = "sink_area_dish_washer_door_handle"

door_name = "sink_area_dish_washer_door"

move_to_the_middle_pose = [2.2, 1.85, 0]
move_to_the_middle_left_side_dishwasher = [2.2, -0.1, 0]
giskardpy.sync_worlds()


def knowledge_interface():
    init_knowrob_interface()
    location = get_location_pose("couch_table")
    print(f"table_pose: {location}")
    #print(f"table_y: {location.posestamped.pose.orientation.y}")
    handle = get_handle_pos(handle_name)
    print(f"handle_pose: {handle}")


with ((real_robot)):
    rospy.loginfo("Starting demo")

    knowledge_interface()




# def navigate_to(turn_around, y, table_name):
#     """
#     Navigates to the popcorntable or to the table on the other side.
#
#     :param x: x pose to navigate to
#     :param y: y pose to navigate to
#     :param orientation: defines the orientation of the robot respectively the name of the table to move to
#     """
# table = get_table_pose(table_name)

#       print(f"table_pose: {table}")
# if turn_around:
#  NavigateAction(target_locations=[
#                   Pose([2.0, y,
#                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#                                                             table.posestamped.pose.orientation.w])]).resolve().perform()
# else:
#     NavigateAction(target_locations=[table.posestamped.pose.position.x, y,
#      #                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#      #                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#      #                                                             table.posestamped.pose.orientation.w])]).resolve().perform()]).resolve().perform()


# lt = LocalTransformer()
#     tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
#     tf_listener = tf2_ros.TransformListener(tf_buffer)
#     transform = tf_buffer.lookup_transform("sink_area_dish_washer_main", "base_link", 0, rospy.Duration(1.0))
#     base_link = PoseStamped()
#     base_link.header.frame_id = "base_link"
#     pose_transformed = tf2_geometry_msgs.do_transform_pose(base_link ,transform)
#     rTb = Pose(frame= "base_link")
#     rTd = lt.transform_pose(rTb, "iai_kitchen/sink_area_dish_washer_main")