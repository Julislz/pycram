from enum import Enum

import rospy
from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib

<<<<<<< HEAD
=======
from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_hsrb_real_test_demos.utils.misc import *
from demos.pycram_hsrb_real_test_demos.utils.misc import sort_objects
# from pycram.external_interfaces.knowrob import get_table_pose
import pycram.helper as helper
import rospkg


# publish robot on param server for gripper tool frame
# rospack = rospkg.RosPack()
# name = "hsrb.urdf"
# package_path = rospack.get_path('pycram') + '/resources/' + name
# urdf_string = helper.urdf_to_string(package_path)
# rospy.set_param('robot_description', urdf_string)

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning



#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
giskardpy.init_giskard_interface()
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])


# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# Main interaction sequence with real robot
with ((real_robot)):
    TalkingMotion("Starting demo").resolve().perform()
    rospy.loginfo("Starting demo")

<<<<<<< HEAD
    NavigateAction(target_locations=[Pose([4, 2, 0], [0, 0, 0, 1])]).resolve().perform()
    PlaceGivenObjAction(["Metalbowl"], ["left"], [Pose([4.86, 2, 0.88])], ["top"]).resolve().perform()
=======
    NavigateAction(target_locations=[Pose([1.45, 5, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    object_desig = DetectAction(technique='all').resolve().perform()
    bowl = get_bowl(object_desig)
    PlaceGivenObjAction(["Milkpackja"], ["left"],
                       [Pose([bowl.pose.position.x + 0.2, 5.8, 0.775])], ["top"]).resolve().perform()
    # PlaceGivenObjAction(["Cronybox"], ["left"], [Pose([4.86, 2, 0.88])], ["front"]).resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
    ParkArmsAction([Arms.LEFT]).resolve().perform()
