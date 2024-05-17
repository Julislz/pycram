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


class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x + pose_in_tilt.position.y)

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})


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
        this value can also be set to 'attributes' or 'location' to get the attributes and pose of a human or a bool
        if a seat specified in the sematic map is taken
        """

        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        if desig.technique == 'human' and (desig.state == 'start' or desig.state == None):
            human_pose = queryHuman()
            return human_pose

        elif desig.state == "stop":
            stop_queryHuman()
            return "stopped"

        elif desig.technique == 'location':
            seat = desig.state
            seat_human_pose = seat_queryHuman(seat)
            # print(seat_human_pose[0].attribute[0].split(','))
            # print(seat_human_pose[0].attribute[1])
            if seat == "long_table":
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
                res.append(i.split(',')[1:])

            return res

        elif desig.technique == 'attributes':
            human_pose_attr = attributes_queryHuman()
            counter = 0
            # wait for human to come
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
                if len(list) == 0:
                    continue
                obj_pose = Pose.from_pose_stamped(list[0])
                # obj_pose.orientation = [0, 0, 0, 1]
                # obj_pose_tmp = query_result.res[i].pose[0]
                obj_type = i.type
                obj_size = i.size
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

                    xvalue = input_string[(input_string.find("x") + 2): input_string.find("y")]
                    y_value = input_string[(input_string.find("y") + 2): input_string.find("z")]
                    z_value = input_string[(input_string.find("z") + 2):]

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

                x, y, z = extract_xyz_values(obj_size)
                # size = (x, z / 2, y)
                # size_box = (x / 2, z / 2, y / 2)
                hard_size = (0.02, 0.02, 0.03)
                # TODO: this needs to work again to be able to run the demos

                # id = BulletWorld.current_bullet_world.add_rigid_box(obj_pose, hard_size, [0, 0, 0, 1])
                # box_object = Object(obj_type + "" + str(rospy.get_time()), obj_type, pose=obj_pose, color=[0, 0, 0, 1], id=id,
                #                     customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
                box_object = Object(obj_type + "" + str(rospy.get_time()), ObjectType.BOWL, "bowl.stl",
                                    pose=Pose([2.5, 2.2, 1.02]),
                                    color=Color(1, 1, 0, 1))
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
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
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
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       "map")
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

    def talk(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBTalkReal(self._talk_lock)

    def pour(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBPourReal(self._pour_lock)

    def head_follow(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBHeadFollowReal(self._head_follow_lock)

    def pointing(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBPointingReal(self._pointing_lock)
