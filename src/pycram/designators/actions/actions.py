import abc
import inspect
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from tf import transformations
from typing_extensions import Union, Type

from ..action_designator import MoveTorsoAction, NavigateAction
from ... import helper
from ...designator import ActionDesignatorDescription
from ..motion_designator import *
from ...datastructures.pose import Pose
from ...datastructures.enums import Arms, Grasp
from ...language import Monitor
from ...ros.force_torque_sensor import ForceTorqueSensor
from ...task import with_tree
from dataclasses import dataclass, field
from ..location_designator import CostmapLocation
from ..object_designator import BelieveObject
from ...helper import multiply_quaternions, axis_angle_to_quaternion
from ...local_transformer import LocalTransformer
from ...orm.base import Pose as ORMPose
from ...orm.object_designator import Object as ORMObject
from ...orm.action_designator import Action as ORMAction
from ...plan_failures import ObjectUnfetchable, ReachabilityFailure, SensorMonitoringCondition
from ...robot_descriptions import robot_description
from ...orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                      PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                      MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                      LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                      TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                      CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                      FaceAtAction as ORMFaceAtAction)
from ...world import World


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable actions."""
    orm_class: Type[ORMAction] = field(init=False, default=None)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def perform(self) -> None:
        """
        Perform the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # list of parameters that will be passed to the ORM class. If the name does not match the orm_class equivalent
        # or if it is a type that needs to be inserted into the session manually, it will not be added to the list
        parameters = [value for key, value in class_variables.items() if key in orm_class_variables
                      and not isinstance(value, (ObjectDesignatorDescription.Object, Pose))]

        return self.orm_class(*parameters)

    def insert(self, session: Session, **kwargs) -> Action:
        """
        Insert this action into the database.

        Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
        equivalent. In that case, the attributes need to be inserted into the session manually.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM action that was inserted into the database
        """

        action = super().insert(session)

        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # loop through all class parameters and insert them into the session unless they are already added by the ORM
        for key, value in class_variables.items():
            if key not in orm_class_variables:
                variable = value.insert(session)
                if isinstance(variable, ORMObject):
                    action.object = variable
                elif isinstance(variable, ORMPose):
                    action.pose = variable
        session.add(action)

        return action


@dataclass
class MoveTorsoActionPerformable(ActionAbstract):
    """
    Move the torso of the robot up and down.
    """

    position: float
    """
    Target position of the torso joint
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMMoveTorsoAction)

    @with_tree
    def perform(self) -> None:
        MoveJointsMotion([robot_description.torso_joint], [self.position]).perform()


@dataclass
class SetGripperActionPerformable(ActionAbstract):
    """
    Set the gripper state of the robot.
    """

    gripper: str
    """
    The gripper that should be set 
    """
    motion: str
    """
    The motion that should be set on the gripper
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMSetGripperAction)

    @with_tree
    def perform(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: str

    object_designator: ObjectDesignatorDescription.Object

    def perform(self) -> None:
        raise NotImplementedError


@dataclass
class GripActionPerformable(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """

    gripper: str
    object_designator: ObjectDesignatorDescription.Object
    effort: float

    @with_tree
    def perform(self) -> None:
        raise NotImplementedError()


@dataclass
class ParkArmsActionPerformable(ActionAbstract):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMParkArmsAction)

    @with_tree
    def perform(self) -> None:
        # create the keyword arguments
        kwargs = dict()
        left_poses = None
        right_poses = None

        # add park left arm if wanted
        if self.arm in [Arms.LEFT, Arms.BOTH]:
            kwargs["left_arm_config"] = "park"
            left_poses = robot_description.get_static_joint_chain("left", kwargs["left_arm_config"])

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            right_poses = robot_description.get_static_joint_chain("right", kwargs["right_arm_config"])

        MoveArmJointsMotion(left_poses, right_poses).perform()


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be picked up
    """

    arm: str
    """
    The arm that should be used for pick up
    """

    grasp: str
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BWorld object is changed.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    @with_tree
    def perform(self) -> None:
        # Initialize the local transformer and robot reference
        lt = LocalTransformer()
        robot = World.robot
        # Retrieve object and robot from designators
        object = self.object_designator.world_object
        # Calculate the object's pose in the map frame
        oTm = object.get_pose()
        execute = True

        # Adjust object pose for top-grasping, if applicable
        if self.grasp == "top":
            print("Metalbowl from top")
            # Handle special cases for certain object types (e.g., Cutlery, Metalbowl)
            # Note: This includes hardcoded adjustments and should ideally be generalized
            if self.object_designator.type == "Cutlery":
                # todo: this z is the popcorn-table height, we need to define location to get that z otherwise it
                #  is hardcoded
                oTm.pose.position.z = 0.71
            oTm.pose.position.z += 0.035

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = grasp_rotation
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        # Open the gripper before picking up the object
        rospy.logwarn("Opening Gripper")
        MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

        # Move to the pre-grasp position and visualize the action
        rospy.logwarn("Picking up now")
        World.current_world.add_vis_axis(oTmG)
        # Execute Bool, because sometimes u only want to visualize the poses to test things
        if execute:
            MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).resolve().perform()
        # Calculate and apply any special knowledge offsets based on the robot and object type
        # Note: This currently includes robot-specific logic that should be generalized
        tool_frame = robot_description.get_tool_frame(self.arm)
        special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot
        if robot.name == "hsrb":
            if self.grasp == "top":
                if self.object_designator.type == "Metalbowl":
                    special_knowledge_offset.pose.position.y += 0.085
                    special_knowledge_offset.pose.position.x -= 0.03

        push_base = special_knowledge_offset
        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot if we dont generlize this we will have a big list in the end of all robots
        if robot.name == "hsrb":
            z = 0.04
            if self.grasp == "top":
                z = 0.025
                if self.object_designator.type == "Metalbowl":
                    z = 0.044
            push_base.pose.position.z += z
        push_baseTm = lt.transform_pose(push_base, "map")
        special_knowledge_offsetTm = lt.transform_pose(push_base, "map")

        # Grasping from the top inherently requires calculating an offset, whereas front grasping involves
        # slightly pushing the object forward.
        rospy.logwarn("Offset now")
        # m = ManualMarkerPublisher()
        # m.create_marker("pose_pickup", special_knowledge_offsetTm)
        World.current_world.add_vis_axis(special_knowledge_offsetTm)
        if execute:
            MoveTCPMotion(special_knowledge_offsetTm, self.arm, allow_gripper_collision=False).resolve().perform()

        rospy.logwarn("Pushing now")
        World.current_world.add_vis_axis(push_baseTm)
        if execute:
            MoveTCPMotion(push_baseTm, self.arm, allow_gripper_collision=False).resolve().perform()

        # Finalize the pick-up by closing the gripper and lifting the object
        rospy.logwarn("Close Gripper")
        MoveGripperMotion(motion="close", gripper=self.arm, allow_gripper_collision=True).resolve().perform()

        rospy.logwarn("Lifting now")
        liftingTm = push_baseTm
        liftingTm.pose.position.z += 0.03
        World.current_world.add_vis_axis(liftingTm)
        if execute:
            MoveTCPMotion(liftingTm, self.arm, allow_gripper_collision=False).resolve().perform()
        tool_frame = robot_description.get_tool_frame(self.arm)
        robot.attach(object=self.object_designator.world_object, link=tool_frame)


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be place
    """
    arm: str
    """
    Arm that is currently holding the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    @with_tree
    def perform(self) -> None:

        def monitor_func():
            fts = ForceTorqueSensor(robot_name='hsrb')
            pr = True
            der: WrenchStamped() = fts.get_last_value()
            print(abs(der.wrench.force.y))
            if abs(der.wrench.force.y) > 0.45:
                print(abs(der.wrench.force.y))
                print(abs(der.wrench.torque.y))
                return SensorMonitoringCondition
            return False

        lt = LocalTransformer()
        robot = World.robot
        execute = True
        # oTm = Object Pose in Frame map
        oTm = self.target_location

        if self.grasp == "top":
            oTm.pose.position.z += 0.05

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = grasp_rotation
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        rospy.logwarn("Placing now")
        World.current_world.add_vis_axis(oTmG)
        if execute:
            MoveTCPMotion(oTmG, self.arm).resolve().perform()

        tool_frame = robot_description.get_tool_frame(self.arm)
        push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
        if robot.name == "hsrb":
            z = 0.03
            if self.grasp == "top":
                z = 0.07
            push_base.pose.position.z += z
        # todo: make this for other robots
        push_baseTm = lt.transform_pose(push_base, "map")

        rospy.logwarn("Pushing now")
        World.current_world.add_vis_axis(push_baseTm)
        if execute:
            MoveTCPMotion(push_baseTm, self.arm).resolve().perform()
        if self.object_designator.type == "Metalplate":
            # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
            rospy.logwarn("sidepush monitoring")
            TalkingMotion("sidepush.").resolve().perform()
            side_push = Pose(
                [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.05, push_baseTm.pose.position.z],
                [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                 push_baseTm.orientation.w])
            try:
                plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.logwarn("Open Gripper")
                MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()


@dataclass
class PlaceGivenObjActionPerformable(ActionAbstract):
    """
    A class representing a designator for a place action of human given objects, allowing a robot to place a
    human given object, that could not be picked up or were not found in the FOV.

    This class encapsulates the details of the place action of human given objects, including the type of the object to
    be placed, the arm to be used, the target_location to place the object and the grasp type. It defines the sequence
    of operations for the robot to execute the place action of human given object, such as moving the arm holding the
    object to the target_location, opening the gripper, and lifting the arm.
    """

    object_type: str
    """
    Object type describing the object that should be placed
    """
    arm: str
    """
    Arm that is currently holding the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    grasp: str
    """
    Grasp that defines how to place the given object
    """
    on_table: Optional[bool]
    """
    When placing a plate needed to differentiate between placing in a dishwasher and placing on the table. 
    Default is placing on a table.
    """

    @with_tree
    def perform(self) -> None:
        lt = LocalTransformer()
        robot = World.robot
        # oTm = Object Pose in Frame map
        oTm = self.target_location

        # TODO add for other robots
        if self.object_type == "Metalplate" and robot.name == "hsrb":

            grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            MoveTCPMotion(oTmG, self.arm).resolve().perform()
            if self.on_table:
                MoveTorsoAction([0.62]).resolve().perform()
                kwargs = dict()

                # taking in the predefined arm configuration for placing
                if self.arm in ["left", "both"]:
                    kwargs["left_arm_config"] = "place_plate"
                    MoveArmJointsMotion(**kwargs).resolve().perform()

                # turning the gripper downwards to better drop the plate
                MoveJointsMotion(["wrist_flex_joint"], [-0.8]).resolve().perform()

                # correct a possible sloped orientation
                NavigateAction(
                    [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0])]).resolve().perform()

            MoveGripperMotion(motion="open", gripper="left").resolve().perform()

            # Move away from the table
            # todo generalize so that hsr is always moving backwards
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x - 0.1, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()

        # placing everything else except the Metalplate
        else:
            if self.grasp == "top":
                oTm.pose.position.z += 0.05

            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            MoveTCPMotion(oTmG, self.arm).resolve().perform()

            tool_frame = robot_description.get_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == "top":
                    z = 0.07
                push_base.pose.position.z += z
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            rospy.logwarn("Pushing now")
            MoveTCPMotion(push_baseTm, self.arm).resolve().perform()

            rospy.logwarn("Open Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.08
            World.current_world.add_vis_axis(liftingTm)

            MoveTCPMotion(liftingTm, self.arm).resolve().perform()


@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMNavigateAction)

    @with_tree
    def perform(self) -> None:
        MoveMotion(self.target_location).perform()


@dataclass
class TransportActionPerformable(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be transported.
    """
    arm: str
    """
    Arm that should be used
    """
    target_location: Pose
    """
    Target Location to which the object should be transported
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def perform(self) -> None:
        robot_desig = BelieveObject(names=[robot_description.name])
        ParkArmsActionPerformable(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig.resolve(),
                                     reachable_arm=self.arm)
        # Tries to find a pick-up posotion for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm in pose.reachable_arms:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionPerformable(pickup_pose.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, "front").perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve(),
                                        reachable_arm=self.arm).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateActionPerformable(place_loc.pose).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()


@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        LookingMotion(target=self.target).perform()


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    technique: str
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', etc. 
    Or 'all' if all objects should be detected
    """

    object_designator: Optional[ObjectDesignatorDescription.Object] = None
    """
    Object designator loosely describing the object, e.g. only type. 
    """

    state: Optional[str] = None
    """
    The state instructs our perception system to either start or stop the search for an object or human.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    @with_tree
    def perform(self) -> None:
        if self.object_designator:
            object_type = self.object_designator.type
        else:
            object_type = None
        return DetectingMotion(technique=self.technique, object_type=object_type,
                               state=self.state).resolve().perform()


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be opened
    """
    arm: str
    """
    Arm that should be used for opening the container
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
    def perform(self) -> None:
        # GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be closed
    """
    arm: str
    """
    Arm that should be used for closing
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion("open", self.arm, allow_gripper_collision=True).perform()


@dataclass
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """

    arm: str
    """
    The arm that should be used to grasp
    """
    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object Designator for the object that should be grasped
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)

    @with_tree
    def perform(self) -> None:
        # if isinstance(self.object_desig, ObjectPart.Object):
        #     object_pose = self.object_desig.part_pose
        # else:
        #     object_pose = self.object_desig.world_object.get_pose()
        # lt = LocalTransformer()
        # gripper_name = robot_description.get_tool_frame(self.arm)
        #
        # object_pose_in_gripper = lt.transform_pose(object_pose,
        #                                            World.robot.get_link_tf_frame(gripper_name))
        #
        # pre_grasp = object_pose_in_gripper.copy()
        # pre_grasp.pose.position.x -= 0.1
        #
        # MoveTCPMotion(pre_grasp, self.arm).perform()
        # MoveGripperMotion("open", self.arm).perform()
        #
        # MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).perform()
        # MoveGripperMotion("close", self.arm, allow_gripper_collision=True).perform()
        #       # if isinstance(self.object_desig, ObjectPart.Object):
        object_pose = self.object_desig

        # Initialize the local transformer and robot reference
        lt = LocalTransformer()
        robot = World.robot  # Retrieve object and robot from designators
        # Calculate the object's pose in the map frame
        oTm = object_pose
        # Todo only for suturo lab and hsr
        oTm.pose.position.x -= 0.2
        execute = True
        grasp = "front"
        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = grasp_rotation

        object_orientation = axis_angle_to_quaternion([1, 0, 0], 90)
        q2 = [oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w]
        new_qua = helper.multiply_quaternions(object_orientation, q2)

        oTb.pose.orientation.x = new_qua[0]
        oTb.pose.orientation.y = new_qua[1]
        oTb.pose.orientation.z = new_qua[2]
        oTb.pose.orientation.w = new_qua[3]

        tool_frame = robot_description.get_tool_frame(self.arm)
        oTgt = lt.transform_pose(oTb, robot.get_link_tf_frame(tool_frame))
        z = oTgt.pose.position.z
        oTgt.pose.position.z = z - 0.01
        oTmgt = lt.transform_pose(oTgt, "map")
        oTgt.pose.position.z = z - 0.01
        oTmG = lt.transform_pose(oTgt, "map")

        # Open the gripper before picking up the object
        rospy.logwarn("Opening Gripper")
        MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

        # Move to the pre-grasp position and visualize the action
        rospy.logwarn("Picking up now")

        World.current_world.add_vis_axis(oTmgt)
        World.current_world.add_vis_axis(oTmG)
        if execute:
            MoveTCPMotion(oTmgt, self.arm).resolve().perform()  # MoveTCPMotion(oTmG, self.arm).resolve().perform()
        rospy.sleep(5)
        # Open the gripper before picking up the object
        rospy.logwarn("Closing Gripper")
        MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()


@dataclass
class FaceAtPerformable(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """

    orm_class = ORMFaceAtAction

    @with_tree
    def perform(self) -> None:
        # get the robot position
        robot_position = World.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = Pose(robot_position.position_as_list(), orientation)

        # turn robot
        NavigateActionPerformable(new_robot_pose).perform()

        # look at target
        LookAtActionPerformable(self.pose).perform()


@dataclass
class MoveAndPickUpPerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    arm: Arms
    """
    The arm to use
    """

    grasp: Grasp
    """
    The grasp to use
    """

    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()


@dataclass
class HeadFollowPerformable(ActionAbstract):
    state: str
    """
    defines if the robot should start/stop looking at human
    """

    def perform(self) -> None:
        HeadFollowMotion(self.state).resolve().perform()


@dataclass
class PouringActionPerformable(ActionAbstract):
    """
    Designator to let the robot perform a pouring action.
    """

    target_location: Pose
    """
    The Pose the robot should pour into.
    """

    arm: str
    """
    The arm that should be used for cutting.
    """

    direction: str
    """
    The direction that should be used for pouring. For example, 'left' or 'right'.
    """

    angle: float
    """
    the angle to move the gripper to.
    """

    @with_tree
    def perform(self) -> None:
        lt = LocalTransformer()
        robot = World.robot

        # TODO add for other robots
        if robot.name == "hsrb":
            # oTm = Object Pose in Frame map
            if self.direction == "right":
                oTm = Pose(
                    [self.target_location.pose.position.x - 0.008, self.target_location.pose.position.y + 0.095,
                     self.target_location.pose.position.z + 0.13], self.target_location.pose.orientation)  # y + 0.095
                # oTm = Pose([self.target_location.pose.position.x - 0.3, self.target_location.pose.position.y + 0.1,
                # self.target_location.pose.position.z + 0.1], self.target_location.pose.orientation)
            else:
                oTm = Pose(
                    [self.target_location.pose.position.x - 0.008, self.target_location.pose.position.y - 0.15,
                     self.target_location.pose.position.z + 0.13], self.target_location.pose.orientation)
                # oTm = Pose([self.target_location.pose.position.x - 0.3, self.target_location.pose.position.y - 0.1,
                # self.target_location.pose.position.z + 0.1], self.target_location.pose.orientation)
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Pouring now")
            MoveTorsoAction([0.37]).resolve().perform()
            MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).resolve().perform()

            # MoveTorsoAction([0.35]).resolve().perform()

            # NavigateAction(
            # [Pose([robot.get_pose().pose.position.x + 0.2, robot.get_pose().pose.position.y,
            # 0], robot.get_pose().pose.orientation)]).resolve().perform()

            # kwargs = dict()
            #
            # # taking in the predefined arm position for pouring
            # if self.arm in ["left", "both"]:
            #     kwargs["left_arm_config"] = "pour"
            #     MoveArmJointsMotion(**kwargs).resolve().perform()

            PouringMotion(self.direction, self.angle).resolve().perform()

            rospy.sleep(3)

            if self.direction == "right":
                PouringMotion("left", 0).resolve().perform()
            else:
                PouringMotion("right", 0).resolve().perform()

            # Move away from the table
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x - 0.15, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()


@dataclass
class OpenDishwasherPerformable(ActionAbstract):
    """
    Opens a container like object

    Can currently not be used
    """

    handle_name: str
    """
    Name of the handle to grasp for opening
    """

    door_name: str
    """
    Name of the door belonging to the handle
    """

    goal_state_half_open: float
    """
    goal state for opening the door half way
    """

    goal_state_full_open: float
    """
    goal state for opening the door fully
    """

    arm: str
    """
    Arm that should be used for opening the container
    """

    @with_tree
    def perform(self) -> None:
        # TODO: Implement this
        return print("OpenDishwasherPerformable is not implemented yet")
        # MoveGripperMotion("open", self.arm).resolve().perform()
        # GraspingDishwasherHandleMotion(self.handle_name, self.arm).resolve().perform()
        #
        # MoveGripperMotion("close", self.arm).resolve().perform()
        # HalfOpeningDishwasherMotion(self.handle_name, self.goal_state_half_open, self.arm).resolve().perform()
        #
        # MoveGripperMotion("open", self.arm).resolve().perform()
        # MoveArmAroundMotion(self.handle_name, self.arm).resolve().perform()
        #
        # MoveGripperMotion("close", self.arm).resolve().perform()
        # FullOpeningDishwasherMotion(self.handle_name, self.door_name, self.goal_state_full_open,
        #                             self.arm).resolve().perform()
        #
        # ParkArmsAction([self.arm]).resolve().perform()
        # MoveGripperMotion("open", self.arm).resolve().perform()
        # plan = talk | park | gripper_open
        # plan.perform()
