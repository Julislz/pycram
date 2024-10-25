<<<<<<< HEAD
import numpy as np
import rospy
from threading import Lock
from typing import Any

from geometry_msgs.msg import PointStamped

from ..datastructures.dataclasses import Color
from ..datastructures.enums import JointType, State
from ..external_interfaces.navigate import queryPoseNav
from ..external_interfaces.robokudo import stop_queryHuman, seat_queryHuman, queryHuman, attributes_queryHuman, \
    queryRegion, queryEmpty
from ..designators.object_designator import ObjectDesignatorDescription
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion, OpeningMotion, ClosingMotion, TalkingMotion, PouringMotion
from ..robot_descriptions import robot_description
from ..process_module import ProcessModule
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..world import World
from ..world_concepts.world_object import Object


# TODO: HSR is currently not supporting simulation
=======
from threading import Lock
from threading import Lock
from typing import Any

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
# from robokudo_msgs.msg import QueryGoal, QueryResult
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from tmc_msgs.msg import Voice

import pycram.bullet_world_reasoning as btr
from ..designators.motion_designator import *
from ..enums import JointType, ObjectType, State

from ..external_interfaces import giskard_new as giskard
# change to giskard_new as giskard
from ..external_interfaces.ik import request_ik
from ..external_interfaces.robokudo import *
from ..helper import _apply_ik
from ..local_transformer import LocalTransformer
#from ..external_interfaces.navigate import queryPoseNav
from ..process_module import ProcessModule


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of HSRB and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class HSRBNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class HSRBPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        arm = desig.arm
        arm_short = "r" if arm == "right" else "l"

        _move_arm_tcp(target, robot, arm)
        tool_frame = robot_description.get_tool_frame(arm)
        robot.attach(object, tool_frame)


class HSRBPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot
        arm = desig.arm

        # Transformations such that the target position is the position of the object and not the tcp
        object_pose = object.get_pose()
        local_tf = LocalTransformer()
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                robot.get_link_tf_frame(robot_description.get_tool_frame(arm)))
        target_diff = desig.target.to_transform("target").inverse_times(tcp_to_object.to_transform("object")).to_pose()

        _move_arm_tcp(target_diff, robot, arm)
        robot.detach(object)


class HSRBMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        robot.set_joint_state("head_pan_joint", new_pan + current_pan)
        robot.set_joint_state("head_tilt_joint", new_tilt + current_tilt)


class HSRBMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class HSRBDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion.Motion):
        rospy.loginfo("Detecting technique: {}".format(desig.technique))
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis
        if desig.technique == 'all':
            rospy.loginfo("Fake detecting all generic objects")
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            rospy.loginfo("Fake detecting human -> spawn 0,0,0")
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            rospy.loginfo("Fake -> Detecting specific object type")
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))
        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        rospy.loginfo("returning dict objects")
        return object_dict


class HSRBMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class HSRBMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            robot.set_joint_states(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_states(desig.left_arm_poses)


class HSRBMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        robot.set_joint_states(dict(zip(desig.names, desig.positions)))


class HSRBWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class HSRBOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        # _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[1])


class HSRBClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################


class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        # giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        queryPoseNav(designator.target)


<<<<<<< HEAD
=======
class HSRBNavigationSemiReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        # queryPoseNav(designator.target)


class HSRBPickUpReal(ProcessModule):

    def _execute(self, designator: PickUpMotion.Motion) -> Any:
        pass


class HSRBPlaceReal(ProcessModule):

    # def _execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
    #    pass
    def _execute(self, designator: PlaceMotion.Motion) -> Any:
        giskard.avoid_all_collisions()
        giskard.place_objects(designator.object, designator.target, designator.grasp)


>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
<<<<<<< HEAD
        robot = World.robot
=======
        robot = BulletWorld.robot
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
<<<<<<< HEAD
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x + pose_in_tilt.position.y)
=======
        new_tilt = np.arctan2(pose_in_tilt.position.z, np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2))

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})
<<<<<<< HEAD
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})
=======
        
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion.Motion) -> Any:
        """
        specifies the query send to robokudo
        :param desig.technique: if this is set to human the hsr searches for human and publishes the pose
        to /human_pose. returns PoseStamped of Human.
<<<<<<< HEAD
        this value can also be set to 'attributes' or 'location' to get the attributes and pose of a human or a bool
        if a seat specified in the sematic map is taken
=======
        this value can also be set to 'attributes', 'location' or 'region' to get the attributes and pose of a human, a bool
        if a seat specified in the sematic map is taken or to describe where objects should be perceived.

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        """

        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        if desig.technique == 'human' and (desig.state == 'start' or desig.state == None):
            human_pose = queryHuman()
            return human_pose
<<<<<<< HEAD

=======
        elif desig.state == "face":

            res = faces_queryHuman()
            id_dict = {}
            keys = []
            if res.res:
                for ele in res.res:
                    id_dict[int(ele.type)] = ele.pose[0]
                    keys.append(int(ele.type))
                id_dict["keys"] = keys
                return id_dict
            else:
                return []
            return res
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        elif desig.state == "stop":
            stop_queryHuman()
            return "stopped"

        elif desig.technique == 'location':
            seat = desig.state
            seat_human_pose = seat_queryHuman(seat)
<<<<<<< HEAD
            # print(seat_human_pose[0].attribute[0].split(','))
            # print(seat_human_pose[0].attribute[1])
            if seat == "long_table":
=======

            if seat == "long_table" or seat == "popcorn_table":
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                loc_list = []
                for loc in seat_human_pose[0].attribute:
                    print(f"location: {loc}, type: {type(loc)}")
                    loc_list.append(loc)
                print(loc_list)
                return loc_list
                # return seat_human_pose[0].attribute
            # if only one seat is checked
            if seat != "sofa":
                return seat_human_pose[0].attribute[0][9:].split(',')
            # when whole sofa gets checked, a list of lists is returned
            res = []
            for i in seat_human_pose[0].attribute:
<<<<<<< HEAD
                res.append(i.split(',')[1:])

=======
                res.append(i.split(','))

            print(res)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
            return res

        elif desig.technique == 'attributes':
            human_pose_attr = attributes_queryHuman()
            counter = 0
            # wait for human to come
<<<<<<< HEAD
=======
            # TODO: try catch block
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
            while not human_pose_attr.res and counter < 6:
                human_pose_attr = attributes_queryHuman()
                counter += 1
                if counter > 3:
                    TalkingMotion("please step in front of me").resolve().perform()
                    rospy.sleep(2)

            if counter >= 3:
                return "False"

            # extract information from query
            gender = human_pose_attr.res[0].attribute[3][13:19]
            if gender[0] != 'f':
                gender = gender[:4]
            clothes = human_pose_attr.res[0].attribute[1][20:]
            brightness_clothes = human_pose_attr.res[0].attribute[0][5:]
            hat = human_pose_attr.res[0].attribute[2][20:]
            attr_list = [gender, hat, clothes, brightness_clothes]
            return attr_list

<<<<<<< HEAD
        elif desig.technique == 'region':
            region = desig.state
            query_result = queryRegion(region)
            perceived_objects = []
            list = query_result[1].pose
            print(list[0].pose.position)
            print(type(list[0].pose.position))
            print(type(query_result))
            print(len(query_result))
            for i in query_result:
                # this has to be pose from pose stamped since we spawn the object with given header
                list = i.pose
                print("Das ist i: ")
                print(i)
                print(type(i.pose))
=======
        # used when region-filter of robokudo should be used
        elif desig.technique == 'region':
            region = desig.state  # name of the region where should be perceived
            query_result = queryRegion(region)
            perceived_objects = []

            for obj in query_result:
                # this has to be pose from pose stamped since we spawn the object with given header
                list = obj.pose
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                if len(list) == 0:
                    continue
                obj_pose = Pose.from_pose_stamped(list[0])
                # obj_pose.orientation = [0, 0, 0, 1]
                # obj_pose_tmp = query_result.res[i].pose[0]
<<<<<<< HEAD
                obj_type = i.type
                obj_size = i.size
=======
                obj_type = obj.type
                obj_size = obj.size
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                # obj_color = query_result.res[i].color[0]
                color_switch = {
                    "red": [1, 0, 0, 1],
                    "green": [0, 1, 0, 1],
                    "blue": [0, 0, 1, 1],
                    "black": [0, 0, 0, 1],
                    "white": [1, 1, 1, 1],
                    # add more colors if needed
                }

<<<<<<< HEAD
                # olor = color_switch.get(obj_color)
=======
                # color = color_switch.get(obj_color)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                # if color is None:
                # color = [0, 0, 0, 1]

                # atm this is the string size that describes the object but it is not the shape size thats why string
                def extract_xyz_values(input_string):
                    # Split the input string by commas and colon to separate key-value pairs
                    # key_value_pairs = input_string.split(', ')

                    # Initialize variables to store the X, Y, and Z values
                    x_value = None
                    y_value = None
                    z_value = None

<<<<<<< HEAD
=======
                    # todo: for now it is a string again, might be changed back. In this case we need the lower commented out code
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                    xvalue = input_string[(input_string.find("x") + 2): input_string.find("y")]
                    y_value = input_string[(input_string.find("y") + 2): input_string.find("z")]
                    z_value = input_string[(input_string.find("z") + 2):]

<<<<<<< HEAD
                    #
                    # # Iterate through the key-value pairs to extract the values
=======
                    # Iterate through the key-value pairs to extract the values
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                    # for pair in key_value_pairs:
                    #     key, value = pair.split(': ')
                    #     if key == 'x':
                    #         x_value = float(value)
                    #     elif key == 'y':
                    #         y_value = float(value)
                    #     elif key == 'z':
                    #         z_value = float(value)

                    return x_value, y_value, z_value

                x, y, z = extract_xyz_values(obj_size)
                # size = (x, z / 2, y)
                # size_box = (x / 2, z / 2, y / 2)
                hard_size = (0.02, 0.02, 0.03)
<<<<<<< HEAD
                # TODO: this needs to work again to be able to run the demos

                # id = BulletWorld.current_bullet_world.add_rigid_box(obj_pose, hard_size, [0, 0, 0, 1])
                # box_object = Object(obj_type + "" + str(rospy.get_time()), obj_type, pose=obj_pose, color=[0, 0, 0, 1], id=id,
                #                     customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
                box_object = Object(obj_type + "" + str(rospy.get_time()), ObjectType.BOWL, "bowl.stl",
                                    pose=Pose([2.5, 2.2, 1.02]),
                                    color=Color(1, 1, 0, 1))
=======
                id = BulletWorld.current_bullet_world.add_rigid_box(obj_pose, hard_size, [0, 0, 0, 1])
                box_object = Object(obj_type + "" + str(rospy.get_time()), obj_type, pose=obj_pose, color=[0, 0, 0, 1],
                                    id=id,
                                    customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(perceived_objects):
                object_dict[obj.name] = obj
            return object_dict

        else:
            query_result = queryEmpty(ObjectDesignatorDescription(types=[desig.object_type]))
            perceived_objects = []
            for i in range(0, len(query_result.res)):
<<<<<<< HEAD
                # this has to be pose from pose stamped since we spawn the object with given header
                obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                # obj_pose.orientation = [0, 0, 0, 1]
                # obj_pose_tmp = query_result.res[i].pose[0]
                obj_type = query_result.res[i].type
                obj_size = query_result.res[i].shape_size
                # obj_color = query_result.res[i].color[0]
                color_switch = {
                    "red": [1, 0, 0, 1],
                    "green": [0, 1, 0, 1],
                    "blue": [0, 0, 1, 1],
                    "black": [0, 0, 0, 1],
                    "white": [1, 1, 1, 1],
                    # add more colors if needed
                }

                # olor = color_switch.get(obj_color)
                # if color is None:
                # color = [0, 0, 0, 1]

                # atm this is the string size that describes the object but it is not the shape size thats why string
                def extract_xyz_values(input_string):
                    # Split the input string by commas and colon to separate key-value pairs
                    # key_value_pairs = input_string.split(', ')

                    # Initialize variables to store the X, Y, and Z values
                    x_value = None
                    y_value = None
                    z_value = None

                    for key in input_string:
                        x_value = key.dimensions.x
                        y_value = key.dimensions.y
                        z_value = key.dimensions.z

                    #
                    # # Iterate through the key-value pairs to extract the values
                    # for pair in key_value_pairs:
                    #     key, value = pair.split(': ')
                    #     if key == 'x':
                    #         x_value = float(value)
                    #     elif key == 'y':
                    #         y_value = float(value)
                    #     elif key == 'z':
                    #         z_value = float(value)

                    return x_value, y_value, z_value

                x, y, ze = extract_xyz_values(obj_size)
                # size = (x, z / 2, y)
                # size_box = (x / 2, z / 2, y / 2)
                hard_size = (0.02, 0.02, 0.03)
                # TODO: this needs to work again to be able to run the demos
                # id = World.current_world.add_rigid_box(obj_pose, hard_size, [0, 0, 0, 1])
                # box_object = Object(obj_type + "_" + str(rospy.get_time()), obj_type, pose=obj_pose, color=[0, 0, 0, 1], id=id,
                #                     customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
                box_object = Object(obj_type + "" + str(rospy.get_time()), ObjectType.BOWL, "bowl.stl",
                                    pose=Pose([2.5, 2.2, 1.02]),
                                    color=Color(1, 1, 0, 1))
=======
                print("#######################################################")
                print(query_result.res[i])
                try:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    continue

                obj_type = query_result.res[i].type
                obj_size = None
                try:
                     obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass
                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                if desig.object_type:
                    if not desig.object_type.lower() in obj_type.lower():
                        pass
                color_switch = {
                    "red": [1, 0, 0, 1],
                    "yellow": [1, 1, 0, 1],
                    "green": [0, 1, 0, 1],
                    "cyan": [0, 1, 1, 1],
                    "blue": [0, 0, 1, 1],
                    "magenta": [1, 0, 1, 1],
                    "white": [1, 1, 1, 1],
                    "black": [0, 0, 0, 1],
                    "grey": [0.5, 0.5, 0.5, 1],
                    # add more colors if needed
                }

                color = color_switch.get(obj_color)
                if color is None:
                    color = [0, 0, 0, 1]


                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]
                osize = [obj_size.x, obj_size.y, obj_size.z]
                id = BulletWorld.current_bullet_world.add_rigid_box(obj_pose, hsize, color)

                box_object = Object(obj_type + "_" + str(rospy.get_time()), obj_type, pose=obj_pose, color=color, id=id,
                                    customGeom={"size": osize})
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

<<<<<<< HEAD
            # Iterate over the list of objects and store each one in the dictionary
=======
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
            for i, obj in enumerate(perceived_objects):
                object_dict[obj.name] = obj
            return object_dict


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion.Motion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
<<<<<<< HEAD
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       "map")
=======
        giskard_return = giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       "map")
        while not giskard_return:
            rospy.sleep(0.1)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        return State.SUCCEEDED, "Nice"


class HSRBMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion.Motion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class HSRBMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion.Motion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)
        return State.SUCCEEDED, "Nice"


class HSRBMoveGripperReal(ProcessModule):
    """
<<<<<<< HEAD
    Opens or closes the gripper of the real HSRB with the help of Giskard.
    """

    def _execute(self, designator: MoveGripperMotion.Motion) -> Any:
        try:
            from tmc_control_msgs.msg import GripperApplyEffortActionGoal
            rospy.loginfo("Imported tmc_control_msgs")
        except ModuleNotFoundError:
            rospy.logwarn("Could not import GripperApplyEffortActionGoal (tmc) messages, HSRB-Real is not supported.")
            return

        pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                      queue_size=10)
        msg = GripperApplyEffortActionGoal()

        if designator.motion == "open":
            msg.goal.effort = 0.8
        elif designator.motion == "close":
            msg.goal.effort = -0.8
        else:
            rospy.logwarn(f"Invalid motion command: {designator.motion}")
            return

        rate = rospy.Rate(10)
        rospy.sleep(2) if designator.motion == "close" else None

        for _ in range(2):  # Publish the message twice
            pub_gripper.publish(msg)
            rate.sleep()
=======
     Opens or closes the gripper of the real HSRB with the help of giskard.
     """

    def _execute(self, designator: MoveGripperMotion.Motion) -> Any:
        if (designator.motion == "open"):
            pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                          queue_size=10)
            rate = rospy.Rate(10)
            rospy.sleep(2)
            msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
            msg.goal.effort = 0.8
            pub_gripper.publish(msg)

        elif (designator.motion == "close"):
            pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                          queue_size=10)
            rate = rospy.Rate(10)
            rospy.sleep(2)
            msg = GripperApplyEffortActionGoal()
            msg.goal.effort = -0.8
            pub_gripper.publish(msg)

        # if designator.allow_gripper_collision:
        #     giskard.allow_gripper_collision("left")
        # giskard.achieve_gripper_motion_goal(designator.motion)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


class HSRBOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion.Motion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion.Motion) -> Any:
        giskard.achieve_close_container_goal(robot_description.get_tool_frame(designator.arm),
                                             designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: TalkingMotion.Motion) -> Any:
<<<<<<< HEAD
        try:
            from tmc_msgs.msg import Voice
            rospy.loginfo("Imported tmc_msgs")
        except ModuleNotFoundError:
            rospy.logwarn("Could not import Voice messages from tmc_msgs, HSRB-Real is not supported.")
            return

        pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # Create and fill the Voice message
        text_to_speech = Voice()
        text_to_speech.language = 1  # 1 = English, 0 = Japanese
        text_to_speech.sentence = designator.cmd

        rospy.sleep(1)
        pub.publish(text_to_speech)
=======
        pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # fill message of type Voice with required data:
        texttospeech = Voice()
        # language 1 = english (0 = japanese)
        texttospeech.language = 1
        texttospeech.sentence = designator.cmd

        rospy.sleep(1)
        pub.publish(texttospeech)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


class HSRBPourReal(ProcessModule):
    """
    Tries to achieve the pouring motion
    """

    def _execute(self, designator: PouringMotion.Motion) -> Any:
        giskard.achieve_tilting_goal(designator.direction, designator.angle)


class HSRBHeadFollowReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: HeadFollowMotion.Motion) -> Any:
        if designator.state == 'stop':
            giskard.stop_looking()
        else:
            giskard.move_head_to_human()


class HSRBPointingReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: PointingMotion.Motion) -> Any:
        pointing_pose = PointStamped()
        pointing_pose.header.frame_id = "map"
        pointing_pose.point.x = designator.x_coordinate
        pointing_pose.point.y = designator.y_coordinate
        pointing_pose.point.z = designator.z_coordinate
        giskard.move_arm_to_pose(pointing_pose)


<<<<<<< HEAD
=======
class HSRBOpenDoorReal(ProcessModule):
    """
    HSR will perform open action on grasped handel for a door
    """

    def _execute(self, designator: DoorOpenMotion.Motion) -> Any:
        giskard.open_doorhandle(designator.handle)


class HSRBGraspHandleReal(ProcessModule):
    """
    HSR will grasp given (door-)handle
    """

    def _execute(self, designator: GraspHandleMotion.Motion) -> Any:
        giskard.grasp_doorhandle(designator.handle)


class HSRBGraspDishwasherHandleReal(ProcessModule):
    """Grasps the dishwasher handle"""

    def _execute(self, designator: GraspingDishwasherHandleMotion.Motion) -> Any:
        giskard.grasp_handle(designator.handle_name)


class HSRBHalfOpenDishwasherReal(ProcessModule):
    """Partially opens the dishwasher door."""

    def _execute(self, designator: HalfOpeningDishwasherMotion.Motion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame("left"), designator.handle_name,
                                            goal_state=designator.goal_state_half_open, special_door=True)


class HSRBMoveArmAroundDishwasherReal(ProcessModule):
    """Moves the HSR arm around the dishwasher door after partially opening"""

    def _execute(self, designator: MoveArmAroundMotion.Motion) -> Any:
        giskard.set_hsrb_dishwasher_door_around(designator.handle_name)


class HSRBFullOpenDishwasherReal(ProcessModule):
    """Opens the dishwasher fully"""

    def _execute(self, designator: FullOpeningDishwasherMotion.Motion) -> Any:
        giskard.fully_open_dishwasher_door(designator.handle_name, designator.door_name)
        giskard.achieve_open_container_goal(robot_description.get_tool_frame("left"), designator.handle_name,
                                            goal_state=designator.goal_state_full_open, special_door=True)


>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
class HSRBManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("hsrb")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._grasp_dishwasher_lock = Lock()
        self._move_around_lock = Lock()
        self._half_open_lock = Lock()
        self._full_open_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()
        self._pour_lock = Lock()
        self._head_follow_lock = Lock()
        self._pointing_lock = Lock()
<<<<<<< HEAD

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == "simulated" or
                ProcessModuleManager.execution_type == "real"):
            return "simulated hsrb is not supported"

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return "simulated hsrb is not supported"
        elif ProcessModuleManager.execution_type == "real":
            return HSRBCloseReal(self._close_lock)
=======
        self._open_door_lock = Lock()
        self._grasp_handle_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBNavigationSemiReal(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBPickUp(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBPickUpReal(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPickUpReal(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBPlace(self._place_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBPlaceReal(self._place_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPlaceReal(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def grasp_dishwasher_handle(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBGraspDishwasherHandleReal(self._grasp_dishwasher_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBGraspDishwasherHandleReal(self._grasp_dishwasher_lock)

    def half_open_dishwasher(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBHalfOpenDishwasherReal(self._half_open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBHalfOpenDishwasherReal(self._half_open_lock)

    def move_arm_around_dishwasher(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmAroundDishwasherReal(self._move_around_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveArmAroundDishwasherReal(self._move_around_lock)

    def full_open_dishwasher(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBFullOpenDishwasherReal(self._full_open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBFullOpenDishwasherReal(self._full_open_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBCloseReal(self._close_lock)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def talk(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBTalkReal(self._talk_lock)
<<<<<<< HEAD
=======
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBTalkReal(self._talk_lock)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def pour(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBPourReal(self._pour_lock)
<<<<<<< HEAD
=======
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPourReal(self._pour_lock)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def head_follow(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBHeadFollowReal(self._head_follow_lock)
<<<<<<< HEAD
=======
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBHeadFollowReal(self._head_follow_lock)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def pointing(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBPointingReal(self._pointing_lock)
<<<<<<< HEAD
=======
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPointingReal(self._pointing_lock)

    def door_opening(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBOpenDoorReal(self._open_door_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBOpenDoorReal(self._open_door_lock)

    def grasp_door_handle(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBGraspHandleReal(self._grasp_handle_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBGraspHandleReal(self._grasp_handle_lock)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
