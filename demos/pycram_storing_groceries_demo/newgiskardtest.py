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
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
giskardpy.clear()

# #robokudo.init_robokudo_interface()
# #rospy.sleep(2)
# giskardpy.spawn_kitchen()
#
# # Shelf Variables
# shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
# lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# shelves = [lower_compartment, middle_compartment, upper_compartment]



shelf_pose = Pose([ 6.388613211499582, 5.896351795034112, 0.0], [0.0, 0.0, 1, 0])
rotated_shelf_pose = Pose([ 6.388613211499582, 5.896351795034112, 0.0], [0.0, 0.0, 0, 1])
# rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
#                           [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
table_pose_pre = Pose([6.7, 4.6,0.0], [0.0, 0.0, 1, 0])

# List of objects
objects = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass", "Orange",
    "Coffeepack", "Softball", "Metalplate", "Pringleschipscan", "Strawberry",
    "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug", "Abrasivesponge",
    "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox",
    "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan",
    "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan", "Peach",
    "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
    "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small",
    "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox*"
]

popcorn_frame = "dinner_table:dinner_table:table_front_edge_center"
# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen",
                                 "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                    "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                    "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                    "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"]
}

# 'shelf:shelf:shelf_floor_0',
links_from_shelf = ['shelf_hohc:shelf_hohc:shelf_floor_0', 'shelf_hohc:shelf_hohc:shelf_floor_1', 'shelf_hohc:shelf_hohc:shelf_floor_2']

giskardpy.sync_worlds()


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def multiply_quaternions(q1, q2):
    """
    Multiply two quaternions.

    Parameters:
    q1 (tuple): First quaternion (x1, y1, z1, w1).
    q2 (tuple): Second quaternion (x2, y2, z2, w2).

    Returns:
    tuple: The product of the two quaternions.
    """

    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def demo(step):
    with real_robot:
        giskardpy.clear()

        # Wait for the start signal
        gripper.pub_now("close")
        pakerino()

        locationtoplace = Pose([2.6868790796016738, 5.717920690528091, 0.715])

        # #giskardpy.spawn_kitchen()
        groups_in_shelf = {
            "Kitchen Utensils and Tools": None,
            "Containers and Drinkware": None,
            "Cleaning Supplies": None,
            "Packaged Food and Beverages": None,
            "Fruits": None,
            "Sports Equipment": None,
            "Miscellaneous": None
        }
        groups_on_table = {}

        if step <= 3:
            talk.pub_now("driving", True)
            move.pub_now(table_pose)

            look_pose = kitchen.get_link_pose(popcorn_frame)
            #look_pose.pose.position.y += 0.7
            perceive_conf = {
                'arm_lift_joint': 0.30,
                'wrist_flex_joint': 1.8,
                'arm_roll_joint': -1,
            }
            pakerino(config=perceive_conf)
            #giskardpy.move_head_to_pose(locationtoplace)
            look_pose.pose.position.z -= 0.05
            giskardpy.move_head_to_pose(look_pose)
            # LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
            # LookAtAction(targets=[look_pose]).resolve().perform()
            talk.pub_now("perceiving", True)
            try:
                table_obj = DetectAction(technique='all').resolve().perform()
                first, *remaining = table_obj
                for dictionary in remaining:
                    for value in dictionary.values():
                        print(value.type)
                        try:
                            group = find_group(value.type)
                            groups_on_table[value.name] = [value, group]

                        except AttributeError:
                            pass
            except PerceptionObjectNotFound:
                rospy.logwarn("Could not find any objects")

                talk.pub_now("I am Done, i hope i did good!")
                pass

        if groups_on_table:
            lt = LocalTransformer()
            groups_on_table_w_table_frame = {}
            for key, obj in groups_on_table.items():
                obj_pose = obj[0].bullet_world_object.pose
                tf_link = kitchen.get_link_tf_frame(popcorn_frame)
                oTb = lt.transform_pose(obj_pose, tf_link)
                grasp_set = None
               # if oTb.pose.position.x >= 0.10:
                    # grasp_set = "top"

                groups_on_table_w_table_frame[key] = (obj[0], oTb, grasp_set)

            # Sort the dictionary by the x coordinate of the transformed pose
            sorted_groups = sorted(groups_on_table_w_table_frame.items(), key=lambda item: item[1][1].position.x)
            # Convert sorted list of tuples back to a dictionary if needed
            sorted_dict = dict(sorted_groups)

            giskardpy.sync_worlds()
            # print(groups_on_table.keys())
            # Initialize an empty dictionary to store the data
            object_data = {}
            # pakerino()

            if sorted_dict:
                first_key, first_value = next(iter(sorted_dict.items()))
                first_object = first_value[0]
                oTb = first_value[1]
                grasp_set = first_value[2]
                id = first_object.bullet_world_object.id
                object_name = first_object.bullet_world_object.name
                object = first_object.bullet_world_object
                # Calculate the object's pose in the map frame
                angle = helper.quaternion_to_angle((oTb.pose.orientation.x, oTb.pose.orientation.y,
                                                    oTb.pose.orientation.z, oTb.pose.orientation.w))
                object_dim = object.get_object_dimensions()

                print("obj dim von " + str(object_name) + str(object_dim))
                graps_set = None
                if grasp_set:
                    grasp = "top"
                else:
                    if object_dim[2] < 0.055:
                        rospy.logwarn(object_name + " grasp is set to top, angle: " + str(angle))
                        rospy.logwarn(object_name + " and height " + str(object_dim[2]))
                        rospy.logwarn(object_name + " and width " + str(object_dim[0]))
                        grasp = "top"
                    # todome you will have to check prerobocup if 0 or 1 here important is
                    elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.070 and object_dim[1] > 0.070):
                        rospy.logwarn(object_name + " grasp is set to top, angle: " + str(angle))
                        rospy.logwarn(object_name + " and height " + str(object_dim[2]))
                        rospy.logwarn(object_name + " and width " + str(object_dim[0]))
                        grasp = "top"
                    else:
                        rospy.logwarn(object_name + " grasp is set to front, angle: " + str(angle))
                        rospy.logwarn(object_name + " and height " + str(object_dim[2]))
                        rospy.logwarn(object_name + " and width " + str(object_dim[0]))
                        grasp = "front"

                    if grasp == "top":
                        print("pose adjusted with z")
                        oTb.pose.position.z += (object_dim[2] / 10)
                        if object_dim[2] < 0.02:
                            rospy.logwarn(" I am not able to grasp the object: " + object_name + " please help me!")
                            oTb.pose.position.z = 0.011
                    else:
                        oTb.pose.position.x += 0.03

                grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
                if grasp == "top":
                    grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
                    oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
                else:
                    oTb.orientation = grasp_rotation
                # noteme oTb is basicly the normal grasp
                oTmG = lt.transform_pose(oTb, "map")

                after_pose = oTmG.copy()
                after_pose.pose.position.z += 0.02

                z_color = [1, 0, 1, 1]
                BulletWorld.current_bullet_world.add_vis_axis(after_pose, z_color=z_color)
                BulletWorld.current_bullet_world.add_vis_axis(oTmG)
                #move.pub_now(table_pose_pre)
                if grasp == "front":
                    config_for_placing = {'arm_lift_joint': -1,
                                          'arm_flex_joint': -0.16,
                                          'arm_roll_joint': -0.0145,
                                          'wrist_flex_joint': -1.417,
                                          'wrist_roll_joint': 0.0}
                else:
                    config_for_placing = {
                        'arm_flex_joint': -1.1,
                        'arm_lift_joint': 1.15,
                        'arm_roll_joint': 0,
                        # 'head_pan_joint': 0.1016423434842566,
                        # 'head_tilt_joint': 0.0255536193297764,
                        'wrist_flex_joint': -1.6,
                        'wrist_roll_joint': 0,
                    }

                pakerino(config=config_for_placing)

                gripper.pub_now("open")
                talk.pub_now("Pick Up now! " + object_name.split('_')[0] + "from:  " + str(grasp))
                giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)

                giskardpy.achieve_attached(object)
                tip_link = 'hand_gripper_tool_frame'
                BulletWorld.robot.attach(object=object, link=tip_link)
                gripper.pub_now("close")
                giskardpy.avoid_all_collisions()
                park = pakerino()
                while not park:
                    print("waiting for park")
                    rospy.sleep(0.1)
                if grasp_listener.check_grasp():
                    talk.pub_now("Vanessas functions says i grasped a object")
                else:
                    talk.pub_now("Vanessas functions says i  was not able to grasped a object")

                gripper.pub_now("open")
                BulletWorld.robot.detach_all()
                world.current_bullet_world.remove_object_w_id([id])
                giskardpy.clear()
                demo(0)



demo(0)
