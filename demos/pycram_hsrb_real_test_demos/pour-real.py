from itertools import islice

import numpy as np
import rospy
import tf
from dynamic_reconfigure.srv import ReconfigureRequest
from geometry_msgs.msg import PointStamped, Twist

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType, ImageEnum
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, HSRBMoveGripperReal, pakerino
from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest


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
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "robocup123.urdf")
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
img = ImageSwitchPublisher()

# 'shelf:shelf:shelf_floor_0',
links_from_shelf = ['shelf_hohc:kitchen_cabinet:shelf_floor_1', 'shelf_hohc:kitchen_cabinet:shelf_floor_2']


table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
table_pose_rotate = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0.7, 0.7])

shelf_1 = Pose([6.35, 5.8, 0],[0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.35, 5.8, 0],[0, 0, -0.7, 0.7])


pose_dishwasher = Pose([9.56, 5.78, 0])
pose_dishwasher_rotate = Pose([9.56, 5.78, 0], [0, 0, 1, 0])

table_new_pose = Pose([8.77, 4.222, 0], [0, 0, 1, 0])
table_new_pose_rotated = Pose([8.77, 4.222, 0], [0, 0, 1, 0])



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

def set_parameters(new_parameters):
    rospy.wait_for_service('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters')
    try:
        reconfigure_service = rospy.ServiceProxy('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters',
                                                 Reconfigure)
        config = Config()

        # Set the new parameters
        if 'forbid_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='forbid_radius', value=new_parameters['forbid_radius']))
        if 'obstacle_occupancy' in new_parameters:
            config.ints.append(IntParameter(name='obstacle_occupancy', value=new_parameters['obstacle_occupancy']))
        if 'obstacle_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='obstacle_radius', value=new_parameters['obstacle_radius']))

        # Empty parameters that are not being set
        config.bools.append(BoolParameter(name='', value=False))
        config.strs.append(StrParameter(name='', value=''))
        config.groups.append(GroupState(name='', state=False, id=0, parent=0))

        req = ReconfigureRequest(config=config)
        reconfigure_service(req)
        rospy.loginfo("Parameters updated successfully")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def move_vel(speed, distance, isForward, angle=0):
    # Starts a new node
    velocity_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    vel_msg = Twist()

    # Checking if the movement is forward or backwards
    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -(abs(speed))
    if angle > 0:
        vel_msg.angular.z = angle
    else:
        vel_msg.angular.z = 0
    # Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0


    # Setting the current time for distance calculation
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # Loop to move the turtle a specified distance
    while not rospy.is_shutdown() and current_distance < distance:
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Take actual time to velocity calculation
        t1 = rospy.Time.now().to_sec()
        # Calculate distance
        current_distance = speed * (t1 - t0)

    # After the loop, stop the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)
# List of objects
objects = ["Fork", "Spoon", "Knife", "Cerealbox", "Metalbowl", "Milkpackja", "Mueslibox", "Cornflakes"]

no_go_objects =["Vase", "Strawberry", "Wineglass", "Scissors", "screwdriver", "hammer", "Washcloth", "Candle", "wooden_block",
                "Peach", "Plum"]


# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife", "Scissors", "screwdriver", "clamp", "hammer"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen", "Cup_small", "Vase"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner", "Sponge", "Washcloth", "Soap"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack", "Pringleschipscan", "Jellobox",
                                    "Dishwashertab", "Cerealbox", "Sugarbox", "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan",
                                    "Gelatinebox", "Mueslibox", "Cornybox", "Masterchefcan", "Pottedmeatcan", "Fantacan", "IceTeaCan",
                                    "Dubbelfris", "Waterbottle", "Colabottle", "Peasoupcan", "Cornflakes", "Curry", "Mayonaise", "Hagelslag",
                                    "Sausages", "Pancakemix", "Crispsbag", "Candy", "Stroopwafels", "Liquorice", "Tictac"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "wooden_block", "Candle", "Grocerybag"]
}


# 'shelf:shelf:shelf_floor_0',
popcorn_frame = "dinner_table:dinner_table:table_front_edge_center"


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose, pose_list):
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 + (obj_pose.pose.position.y - pose.position.y) ** 2 + (
                obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def calculate_z_offsets(links_from_shelf):
    """
    Calculate the Z offsets between each link in links_from_shelf and return a dictionary mapping each link
    to the Z height difference to the next link.
    """
    z_offsets = {}

    # Get the pose (including Z position) for each link
    link_poses = {link: kitchen.get_link_pose(link) for link in links_from_shelf}

    # Iterate through the links and calculate Z offsets
    for i, link in enumerate(links_from_shelf[:-1]):  # Skip the last link
        current_pose = link_poses[link]
        next_pose = link_poses[links_from_shelf[i + 1]]
        current_z = current_pose.position.z  # Adjust according to the structure of Pose
        next_z = next_pose.position.z  # Adjust according to the structure of Pose
        z_offset = next_z - current_z
        z_offsets[link] = z_offset

    return z_offsets


def get_z_height_to_next_link(link, z_offsets):
    """
    Return the Z height difference from the given link to the next link.
    """
    return z_offsets.get(link, None)


#noteme if you want to be able to run this in simulation you will have to do get.aabb without bullet world obj
def find_pose_in_shelf(group, object, groups_in_shelf):
    link = None
    nearest_pose_to_group = None
    try:
        link = groups_in_shelf[group][1]
        group_pose = groups_in_shelf[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                          object_desig=object)
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)

    except (TypeError, KeyError):
        try:
            place_poses = []
            for link in links_from_shelf:
                place_poses.append(
                    (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                         object_desig=object), link)
                )

            # in this case it's the biggest group since why not? i assume most poses are then free
            longest_group = max(place_poses, key=lambda x: len(x[0]))
            nearest_pose_to_group = longest_group[0][0]
            link = longest_group[1]
            if group not in groups_in_shelf:
                groups_in_shelf[group] = [nearest_pose_to_group, longest_group[1]]

        except (TypeError, KeyError):
            return False, False
    if nearest_pose_to_group:
        z_offsets = calculate_z_offsets(links_from_shelf)
        z_height_to_next = get_z_height_to_next_link(link, z_offsets)

        pose_in_shelf = lt.transform_pose(nearest_pose_to_group, kitchen.get_link_tf_frame(link))
        pose_in_shelf.pose.position.x = -0.10
        if z_height_to_next:
            pose_in_shelf.pose.position.z = (z_height_to_next / 2) - 0.01
        else:
            pose_in_shelf.pose.position.z += 0.03
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")
        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map, link


previous_value = None


def monitor_func_place():
    global previous_value
    der = fts.get_last_value()
    current_value = fts.get_last_value()

    prev_force_x = previous_value.wrench.force.x
    curr_force_x = current_value.wrench.force.x
    change_in_force_x = abs(curr_force_x - prev_force_x)
    print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

    def calculate_dynamic_threshold(previous_force_x):
        # Placeholder for a dynamic threshold calculation based on previous values
        # This function can be enhanced to calculate a threshold based on the history of values or other logic
        return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

    if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
        print("Significant change detected")

        return SensorMonitoringCondition

    return False


def placeorpark(object_name, object, grasp, talk_bool, target_location, link, pick_up_bool):
    global previous_value
    oTm = target_location
    oTm.pose.position.z += 0.02

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(link))

    if grasp == "top":
        grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
    else:
        oTb.orientation = grasp_rotation

    oTmG = lt.transform_pose(oTb, "map")

    BulletWorld.current_bullet_world.add_vis_axis(oTmG)

    # todome this mshoudl be depending on the height of the shelf tbh

    if grasp == "front":
        config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                              'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
    else:
        config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': 1.15, 'arm_roll_joint': 0,
                              'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

    pakerino(config=config_for_placing)
    if pick_up_bool:
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
    else:
        talk.pub_now(f"Placing now! {object_name.split('_')[0]} from: {grasp}")
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
    while not giskard_return:
        rospy.sleep(0.1)
    config_after_place = {'arm_lift_joint': 0.0}
    talk.pub_now("tracking placning now")
    previous_value = fts.get_last_value()
    try:
        plan = Code(lambda: giskardpy.test(config_after_place)) >> Monitor(monitor_func_place)
        return_plan = plan.perform()

    except Exception as e:
        print(f"Exception type: {type(e).__name__}")

    giskardpy.achieve_attached(object, tip_link="map")
    BulletWorld.robot.detach(object)
    gripper.pub_now("open")

    giskardpy.avoid_all_collisions()
    park = pakerino()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)


# noteme this is with driving! return grasped_bool, grasp, group, object, obj_id, groups_on_table
def process_pick_up_objects(talk_bool):
    # drive

    look_pose = kitchen.get_link_pose("shelf_hohc:kitchen_cabinet:shelf_floor_1")
    # park
    perceive_conf = {'arm_lift_joint': 0.3, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    look_pose.pose.position.z -= 0.03
    giskardpy.move_head_to_pose(look_pose)

    talk.pub_now("perceiving", True)
    img.pub_now(ImageEnum.SEARCH.value)
    groups_on_table = {}

    # noteme detect on table
    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        first, *remaining = table_obj
        for dictionary in remaining:
            for value in dictionary.values():
                try:

                    tmp_name = value.name.split('_')[0]
                    print(tmp_name)
                    if tmp_name not in objects:
                        continue

                    else:
                        group = find_group(value.type)
                        groups_on_table[value.name] = [value, group]
                except AttributeError:
                    pass
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        talk.pub_now("I am Done, I hope I did good!")
        return

    # noteme if groups were found
    if groups_on_table:
        groups_on_table_w_table_frame = {}
        img.pub_now(ImageEnum.PICKING.value)

        for key, obj in groups_on_table.items():
            obj_pose = obj[0].bullet_world_object.pose
            object_raw = obj[0]
            nearest_link = get_closet_link_to_pose(obj_pose)
            tf_link = kitchen.get_link_tf_frame(nearest_link)
            oTb = lt.transform_pose(obj_pose, tf_link)

            grasp_set = None
            # if to far behind the front face were robots look on the table
            if oTb.pose.position.x >= 0.20:
                grasp_set = "top"

            # noteme this saves the group
            try:
                obj[1]
            except KeyError:
                obj[1] = "Unknown"

            groups_on_table_w_table_frame[key] = (obj[0], oTb, grasp_set, obj[1])

        # noteme sorts objects on the table with x so i can pick up the "most nearest"
        sorted_groups = sorted(groups_on_table_w_table_frame.items(), key=lambda item: item[1][1].pose.position.x)
        sorted_dict = dict(sorted_groups)

        # noteme get first items since we sorted it already
        first_key, first_value = next(iter(sorted_dict.items()))
        first_object = first_value[0]
        oTb = first_value[1]
        grasp_set = first_value[2]
        group = first_value[3]
        print(group)
        obj_id = first_object.bullet_world_object.id
        object_name = first_object.bullet_world_object.name
        object = first_object.bullet_world_object
        angle = helper.quaternion_to_angle(
            (oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")
        grasp_set = True
        if grasp_set:
            grasp = "front"
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            elif object_dim[2] < 0.060 or angle > 40 and (object_dim[0] > 0.08 and object_dim[1] > 0.08):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "front"

            if grasp == "top":
                print("pose adjusted with z")
                oTb.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    # rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    oTb.pose.position.z = 0.011
            else:
                oTb.pose.position.x += 0.03

        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        if grasp == "top":
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        else:
            oTb.orientation = grasp_rotation

        oTmG = lt.transform_pose(oTb, "map")
        after_pose = oTmG.copy()
        after_pose.pose.position.z += 0.02

        z_color = [1, 0, 1, 1]
        BulletWorld.current_bullet_world.add_vis_axis(after_pose, z_color=z_color)
        BulletWorld.current_bullet_world.add_vis_axis(oTmG)

        # testme this is not tested yet
        move_pre_pose_object = robot.get_pose()
        move_pre_pose_object.pose.position.y = oTmG.pose.position.y
        move.pub_now(move_pre_pose_object)

        if grasp == "front":
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        pakerino(config=config_for_placing)

        gripper.pub_now("open")
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
        while not giskard_return:
            rospy.sleep(0.1)

        giskardpy.achieve_attached(object)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(object=object, link=tip_link)
        rospy.sleep(2)
        #move_vel(speed=0.5, distance=1, isForward=False)
        gripper.pub_now("close")
        rospy.sleep(2)
        move_vel(speed=0.3, distance=0.6, isForward=False)
        rospy.sleep(2)
        giskardpy.avoid_all_collisions()
        park = pakerino()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        grasped_bool = None
        if grasp_listener.check_grasp():
            talk.pub_now("Grasped a object", False)
            grasped_bool = True
        else:
            talk.pub_now("I was not able to grasped a object", False)
            grasped_bool = False

        return grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table


# noteme  with look  return groups in shelf
def process_objects_in_shelf(talk_bool):
    # perceive shelf
    groups_in_shelf = {
        "Kitchen Utensils and Tools": None,
        "Containers and Drinkware": None,
        "Cleaning Supplies": None,
        "Packaged Food and Beverages": None,
        "Fruits": None,
        "Sports Equipment": None,
        "Miscellaneous": None,
        "Unknown": None
    }
    #todome fix this again to shelf
    look_pose = kitchen.get_link_pose('shelf_hohc:kitchen_cabinet:shelf_floor_1')
    perceive_conf = {'arm_lift_joint': 0.30, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    look_pose.pose.position.z -= 0.0
    giskardpy.move_head_to_pose(look_pose)

    talk.pub_now("perceiving", True)

    shelf_obj = DetectAction(technique='all').resolve().perform()
    first, *remaining = shelf_obj
    for dictionary in remaining:
        for value in dictionary.values():

            try:
                group = find_group(value.type)
                link = get_closet_link_to_pose(value.pose)
                groups_in_shelf[group] = [value.pose, link, value]

            except AttributeError:
                pass
    return groups_in_shelf


groups_in_shelf = {}


def demo(step):
    global groups_in_shelf
    with ((((real_robot)))):
        object_name = None,
        object = None,
        grasp = None,
        talk_bool = None,
        target_location = None,
        link = None
        talk_bool = True
        gripper.pub_now("close")
        pakerino()

        if step <= 1:
            #talk.pub_now("driving", True)
            img.pub_now(ImageEnum.DRIVINGBACK.value)

            move.pub_now(shelf_1, interrupt_bool=False)
            img.pub_now(ImageEnum.SEARCH.value)
            #groups_in_shelf = process_objects_in_shelf(talk_bool)


        if step <= 2:
            grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table = process_pick_up_objects(talk_bool)
            try:
                talk.pub_now("driving", True)
                img.pub_now(ImageEnum.DRIVINGBACK.value)

                grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table = process_pick_up_objects(
                    talk_bool)
            except TypeError:
                print("done")
                return
        if step <= 3:
            talk.pub_now("driving", True)
            img.pub_now(ImageEnum.DRIVINGBACK.value)
            move.pub_now(table_pose_rotate, interrupt_bool=False)
            move.pub_now(shelf_1, interrupt_bool=False)

            img.pub_now(ImageEnum.PLACING.value)
            placeorpark(object.name, object, "front", talk_bool, place_pose, link_new, False)
            demo(2)





#
#
#
# try:
#
#     img.pub_now(ImageEnum.HI.value)  # hi im toya
#     talk.pub_now("Push down my Hand, when you are Ready.")
#     img.pub_now(ImageEnum.PUSHBUTTONS.value)
#     plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
#     plan.perform()
# except SensorMonitoringCondition:
#     talk.pub_now("Starting Storing Grocery.")
#     # load everfyhing world giskard robokudo....
#     # Wait for the start signal
#
#     img.pub_now(ImageEnum.HI.value)
#     start_signal_waiter.wait_for_startsignal()
#
#     # Once the start signal is received, continue with the rest of the script
#     rospy.loginfo("Start signal received, now proceeding with tasks.")
#     move_vel(speed=2, distance=4, isForward=True)
#     rospy.sleep(2)
#     new_parameters = {
#         'forbid_radius': 0.2,
#         'obstacle_occupancy': 5,
#         'obstacle_radius': 0.2
#     }
#
#     set_parameters(new_parameters)    # Once the start signal is received, continue with the rest of the script
#
#     rospy.sleep(1)
#
#     fake_pose_2 = Pose([3, 0.3, 0])
#     move.pub_fake_pose(fake_pose_2)
#     move_vel(0.2, 2, False, 0.06)
#     talk.pub_now("Lets Go, Driving to Shelf.")
#     move_123 = Pose([4, -0.2, 0], [0, 0 , 0.7, 0.7])
#     move.pub_now(move_123)
#     move_145 = Pose([4.8, 0.8, 0], [0, 0, 0.7, 0.7])
#     move.pub_now(move_145)

#rospy.sleep(2)
#move_vel(speed=0.1, distance=1, isForward=True)
#pakerino()
demo(0)
