import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_vanessa.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
grasp_listener = GraspListener()
talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
lt = LocalTransformer()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
giskardpy.clear()

long_table_1 = Pose([6.65, 4.6, 0],[0, 0, 0, 1])
long_table_pick = Pose([6.60, 4.6, 0],[0, 0, 0, 1])

long_table_1_rotated = Pose([6.65, 4.6, 0],[0, 0, 1, 0])
shelf_1 = Pose([6.2, 5.6, 0],[0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.2, 5.6, 0],[0, 0, -0.7, 0.7])
shelf_1_rotated = Pose([6.2, 5.6, 0],[0, 0, 0, 1])

talk.pub_now("start", True)

perceive_conf = {'arm_lift_joint': 0, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
pakerino(config=perceive_conf)
look_pose = kitchen.get_link_pose("shelf_hohc:shelf_hohc:shelf_floor_1")
look_pose.pose.position.z -= 0.05
giskardpy.move_head_to_pose(look_pose)
print(robot.get_pose())
# good perceive look at the top 2
#   stamp:
#     secs: 1721126684
#     nsecs: 140324115
#   frame_id: "map"
# point:
#   x: 5.4236907958984375
#   y: 5.789007186889648
#   z: 0.7400000214576721


#3_floors_good_planning_pose_far.bag
# 3shelfs
# header:
#   seq: 0
#   stamp:
#     secs: 1721126915
#     nsecs: 901983022
#   frame_id: "map"
# point:
#   x: 5.4236907958984375
#   y: 5.789007186889648
#   z: 0.3400000214576721
# header:
#   seq: 0
#   stamp:
#     secs: 1721126926
#     nsecs: 186844587
#   frame_id: "map"
# pose:
#   position:
#     x: 6.172557745672534
#     y: 5.870382349786555
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.998517482081908
#     w: -0.05443195731191911
