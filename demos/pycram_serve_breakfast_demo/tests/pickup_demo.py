from enum import Enum

import rospy
from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib

from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
# from pycram.external_interfaces.knowrob import get_table_pose
import pycram.helper as helper
import rospkg
import pycram.external_interfaces.giskard as giskardpy



# publish robot on param server for gripper tool frame
# rospack = rospkg.RosPack()
# name = "hsrb.urdf"
# package_path = rospack.get_path('pycram') + '/resources/' + name
# urdf_string = helper.urdf_to_string(package_path)
# rospy.set_param('robot_description', urdf_string)

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning



#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_door_open_9.urdf")

giskardpy.init_giskard_interface()
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#giskardpy.removing_of_objects()
giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])


# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)



giskardpy.giskard_wrapper.avoid_all_collisions()


with ((real_robot)):
    TalkingMotion("Starting demo").resolve().perform()
    rospy.loginfo("Starting demo")
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    #print(f"orientation: {robot.get_pose().pose.orientation}, position: {robot.get_pose().pose.positioen}")

    #client = actionlib.ActionClient(server_name='move_base/move', action_name=MoveBaseAction)
    #NavigateAction(target_locations=[Pose([2.24, 1.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    #MoveTorsoAction([0.2]).resolve().perform()
    LookAtAction(targets=[Pose([5.1, 2.1, 0.21], [0, 0, 0, 1])]).resolve().perform()
    object_desig = DetectAction(technique='location', state='long_table').resolve().perform()
    sorted_places = get_free_spaces(object_desig[1])
    # object_desig = DetectAction(technique='all').resolve().perform()
    # sort_objects = sort_objects(object_desig, wished_sorted_obj_list=["Metalbowl", "mueslibox", "Spoon", "Milkpackja"])
    # bowl = get_bowl(object_desig)
    # print(bowl)
    # sort_objects = sort_objects(robot, object_desig, wished_sorted_obj_list=["Spoon"])
    # for value in range(len(sort_objects)):
    #     grasp = "front"
    #     if sort_objects[value].type in ["Spoon", "Fork", "Knife", "Plasticknife"]:
    #         sort_objects[value].type = "Cutlery"
    #     if sort_objects[value].type in ["Mueslibox", "Cerealbox", "Crackerbox"]:
    #         sort_objects[value].type = "Cerealbox"
    #     # if sort_objects[value].type in ["Milkpackja", "Milkpackbaeren"]:
    #         # sort_objects[value].type = "Milkpack"
    #
    #     if sort_objects[value].type in ["Metalbowl", "Cutlery"]:
    #         grasp = "top"
    #         if sort_objects[value].pose.position.z >= 0.65:
    #             sort_objects[value].pose.position.z = 0.715
    #         elif sort_objects[value].pose.position.z >= 0.4:
    #             sort_objects[value].pose.position.z = 0.46
    #         else:
    #             sort_objects[value].pose.position.z = 0.007
    #
    #     TalkingMotion("Picking up with: " + grasp).resolve().perform()
    #     try_pick_up(robot, sort_objects[value], grasp)

        # Move away from the table
        # NavigateAction([Pose([robot.get_pose().pose.position.x - 0.15, robot.get_pose().pose.position.y,
                              #0])]).resolve().perform()

