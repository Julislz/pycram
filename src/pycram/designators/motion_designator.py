from abc import ABC, abstractmethod
from dataclasses import dataclass

from sqlalchemy.orm import Session
from .object_designator import ObjectDesignatorDescription, ObjectPart, RealObject
from ..designator import ResolutionError
from ..orm.base import ProcessMetaData
from ..plan_failures import PerceptionObjectNotFound
from ..process_module import ProcessModuleManager
from ..orm.motion_designator import (MoveMotion as ORMMoveMotion,
                                     MoveTCPMotion as ORMMoveTCPMotion, LookingMotion as ORMLookingMotion,
                                     MoveGripperMotion as ORMMoveGripperMotion, DetectingMotion as ORMDetectingMotion,
                                     OpeningMotion as ORMOpeningMotion, ClosingMotion as ORMClosingMotion,
                                     Motion as ORMMotionDesignator)
from ..datastructures.enums import ObjectType

from typing_extensions import Dict, Optional, get_type_hints, get_args, get_origin
from ..datastructures.pose import Pose
from ..task import with_tree


@dataclass
class BaseMotion(ABC):

    @abstractmethod
    def perform(self):
        """
        Passes this designator to the process module for execution. Will be overwritten by each motion.
        """
        pass
        # return ProcessModule.perform(self)

    @abstractmethod
    def to_sql(self) -> ORMMotionDesignator:
        """
        Create an ORM object that corresponds to this description. Will be overwritten by each motion.

        :return: The created ORM object.
        """
        return ORMMotionDesignator()

    @abstractmethod
    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        """
        Add and commit this and all related objects to the session.
        Auto-Incrementing primary keys and foreign keys have to be filled by this method.

        :param session: Session with a database that is used to add and commit the objects
        :param args: Possible extra arguments
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM motion.
        """
        metadata = ProcessMetaData().insert(session)

        motion = self.to_sql()
        motion.process_metadata = metadata

        return motion

    def __post_init__(self):
        """
        Checks if types are missing or wrong
        """
        right_types = get_type_hints(self)
        attributes = self.__dict__.copy()

        missing = []
        wrong_type = {}
        current_type = {}

        for k in attributes.keys():
            attribute = attributes[k]
            attribute_type = type(attributes[k])
            right_type = right_types[k]
            types = get_args(right_type)
            if attribute is None:
                if not any([x is type(None) for x in get_args(right_type)]):
                    missing.append(k)
            elif attribute_type is not right_type:
                if attribute_type not in types:
                    if attribute_type not in [get_origin(x) for x in types if x is not type(None)]:
                        wrong_type[k] = right_types[k]
                        current_type[k] = attribute_type
        if missing != [] or wrong_type != {}:
            raise ResolutionError(missing, wrong_type, current_type, self.__class__)


@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

<<<<<<< HEAD
    target: Pose
    """
    Location to which the robot should be moved
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.navigate().execute(self)
        # return ProcessModule.perform(self)

    def to_sql(self) -> ORMMoveMotion:
        return ORMMoveMotion()
=======
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object: ObjectDesignatorDescription.Object
        """
        Object designator of the object to be placed
        """
        target: Pose
        """
        Pose at which the object should be placed
        """
        arm: str
        """
        Arm that is currently holding the object
        """
        grasp: str

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.place().execute(self)

    def __init__(self, object_desig: ObjectDesignatorDescription.Object, target: Pose,
                 arm: Optional[str] = None, grasp: str = "front", resolver: Optional[Callable] = None):
        """
        Places the object in object_desig at the position in target. If an arm is given then the arm is used, otherwise
        arm defaults to ``'left'``

        :param object_desig: Object designator describing the object to be placed
        :param target: The target pose on which to place the object
        :param arm: An arm to use for placing
        :param resolver: An alternative resolver that resolves the list of parameters to a resolved motion designator.
        """
        super().__init__(resolver)
        self.cmd: str = 'place'
        self.object_desig: ObjectDesignatorDescription.Object = object_desig
        self.target: Pose = target
        self.arm: str = arm
        self.grasp: str = grasp
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def insert(self, session, *args, **kwargs) -> ORMMoveMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

<<<<<<< HEAD
        return motion
=======
        :return: A resolved performable motion designator
        """
        arm = "left" if not self.arm else self.arm
        return self.Motion(self.cmd, self.object_desig, self.target, arm, self.grasp)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """
    arm: str
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """

<<<<<<< HEAD
    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_tcp().execute(self)
=======
    def __init__(self, target: Pose, arm: Optional[str] = None,
                 resolver: Optional[Callable] = None, allow_gripper_collision: Optional[bool] = False):
        """
        Moves the TCP of the given arm to the given target pose.
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def to_sql(self) -> ORMMoveTCPMotion:
        return ORMMoveTCPMotion(self.arm, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveTCPMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: Pose

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.looking().execute(self)

    def to_sql(self) -> ORMLookingMotion:
        return ORMLookingMotion()

    def insert(self, session: Session, *args, **kwargs) -> ORMLookingMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: str
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: str
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_gripper().execute(self)

    def to_sql(self) -> ORMMoveGripperMotion:
        return ORMMoveGripperMotion(self.motion, self.gripper, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveGripperMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot
    """

<<<<<<< HEAD
    object_type: ObjectType
    """
    Type of the object that should be detected
    """
=======
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        motion: str
        """
        Motion that should be performed, either 'open' or 'close'
        """
        gripper: str
        """
        Name of the gripper that should be moved
        """
        allow_gripper_collision: bool
        """
        If the gripper is allowed to collide with something
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return (pm_manager.move_gripper().execute(self))

        def to_sql(self) -> ORMMoveGripperMotion:
            return ORMMoveGripperMotion(self.motion, self.gripper, self.allow_gripper_collision)

        def insert(self, session: Session, *args, **kwargs) -> ORMMoveGripperMotion:
            motion = super().insert(session)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        world_object = pm_manager.detecting().execute(self)
        if not world_object:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {self.object_type} in the FOV of the robot")
        if ProcessModuleManager.execution_type == "real":
            return RealObject.Object(world_object.name, world_object.obj_type,
                                     world_object, world_object.get_pose())

        return ObjectDesignatorDescription.Object(world_object.name, world_object.obj_type,
                                                  world_object)

    def to_sql(self) -> ORMDetectingMotion:
        return ORMDetectingMotion(self.object_type)

    def insert(self, session: Session, *args, **kwargs) -> ORMDetectingMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class MoveArmJointsMotion(BaseMotion):
    """
    Moves the joints of each arm into the given position
    """

<<<<<<< HEAD
    left_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the left arm joints
    """
    right_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the right arm joints
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_joints().execute(self)
=======
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        technique: str
        """
        Technique means how the object should be detected, e.g. 'color', 'shape', 'all', etc. 
        """

        object_type: Optional[str]
        """
        Type of the object that should be detected
        """

        state: Optional[str] = None
        """
        The state instructs our perception system to either start or stop the search for an object or human.
        Can also be used to describe the region or location where objects are perceived.
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            bullet_world_objects = pm_manager.detecting().execute(self)
            if not bullet_world_objects:
                raise PerceptionObjectNotFound(
                    f"Could not find an object with the type {self.object_type} in the FOV of the robot")
            return bullet_world_objects

    def __init__(self, technique: str, resolver: Optional[Callable] = None, object_type: Optional[str] = None,
                 state: Optional[str] = None):
        """
        Detects an object in the FOV of the robot. If an object type is given then the object will be searched for 

        :param technique: Technique that should be used for detecting, e.g. 'color', 'shape', 'all', etc.
        :param object_type: Type of the object which should be detected
        :param state The state instructs our perception system to either start or stop the search for an object or human.
        :param resolver: An alternative resolver which returns a resolved motion designator
        """
        super().__init__(resolver)
        self.cmd: str = 'detecting'
        self.technique: str = technique
        self.object_type: Optional[str] = object_type
        self.state: Optional[str] = state
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def to_sql(self) -> ORMMotionDesignator:
        pass

<<<<<<< HEAD
    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass
=======
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.technique, self.object_type, self.state)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


@dataclass
class WorldStateDetectingMotion(BaseMotion):
    """
    Detects an object based on the world state.
    """

<<<<<<< HEAD
    object_type: ObjectType
    """
    Object type that should be detected
    """
=======
    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        left_arm_poses: Dict[str, float]
        """
        Target positions for the left arm joints
        """
        right_arm_poses: Dict[str, float]
        """
        Target positions for the right arm joints
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.move_arm_joints().execute(self)

    def __init__(self, left_arm_config: Optional[str] = None, right_arm_config: Optional[str] = None,
                 left_arm_poses: Optional[dict] = None, right_arm_poses: Optional[dict] = None,
                 resolver: Optional[Callable] = None):
        """
        Moves the arm joints, target positions can either be pre-defined configurations (like 'park') or a
        dictionary with joint names as keys and joint positions as values. If a configuration and a
        dictionary are given the dictionary will be preferred.

        :param left_arm_config: Target configuration for the left arm
        :param right_arm_config: Target configuration for the right arm
        :param left_arm_poses: Target Dict for the left arm
        :param right_arm_poses: Target Dict for the right arm
        :param resolver: An alternative resolver that returns a resolved motion designator for the given parameters.
        """
        super().__init__(resolver)
        self.cmd = 'move-arm-joints'
        self.left_arm_config: str = left_arm_config
        self.right_arm_config: str = right_arm_config
        self.left_arm_poses: Dict[str, float] = left_arm_poses
        self.right_arm_poses: Dict[str, float] = right_arm_poses

    def ground(self) -> Motion:
        """
        Default resolver for moving the arms, returns a resolved motion designator containing the target positions for
        the left and right arm joints.
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.world_state_detecting().execute(self)

<<<<<<< HEAD
    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass
=======
        if self.left_arm_poses:
            left_poses = self.left_arm_poses
        elif self.left_arm_config == "park":
            left_poses = robot_description.get_static_joint_chain("left", self.left_arm_config)
        # predefined arm motion for placing human given plate
        elif self.left_arm_config == "place_plate":
            left_poses = robot_description.get_static_joint_chain("placing_pos", self.left_arm_config)
        elif self.left_arm_config == "pick_up_paper":
            left_poses = robot_description.get_static_joint_chain("pick_up_paper_conf", self.left_arm_config)
        elif self.left_arm_config == "open_dishwasher":
            left_poses = robot_description.get_static_joint_chain("open_dishwasher", self.left_arm_config)
        if self.right_arm_poses:
            right_poses = self.right_arm_poses
        elif self.right_arm_config:
            right_poses = robot_description.get_static_joint_chain("right", self.right_arm_config)
        return self.Motion(self.cmd, left_poses, right_poses)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


@dataclass
class MoveJointsMotion(BaseMotion):
    """
    Moves any joint on the robot
    """
<<<<<<< HEAD
=======

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        object_type: str
        """
        Object type that should be detected
        """
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    names: list
    """
    List of joint names that should be moved 
    """
    positions: list
    """
    Target positions of joints, should correspond to the list of names
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: str
    """
    Arm that should be used
    """
<<<<<<< HEAD
=======

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        # cmd: str
        names: list
        """
        List of joint names that should be moved 
        """
        positions: list
        """
        Target positions of joints, should correspond to the list of names
        """
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.open().execute(self)

    def to_sql(self) -> ORMOpeningMotion:
        return ORMOpeningMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMOpeningMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion

<<<<<<< HEAD

@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """
=======
        :return: A resolved motion designator
        """
        if len(self.names) != len(self.positions):
            raise DesignatorError("[Motion Designator][Move Joints] The length of names and positions does not match")
        for i in range(len(self.names)):
            lower, upper = BulletWorld.robot.get_joint_limits(self.names[i])
            if self.positions[i] < lower or self.positions[i] > upper:
                raise DesignatorError(
                    f"[Motion Designator][Move Joints] The given configuration for the Joint {self.names[i]} violates its limits: (lower = {lower}, upper = {upper})")
        return self.Motion(self.cmd, self.names, self.positions)

class GraspingDishwasherHandleMotion(MotionDesignatorDescription):
    """
    Designator for grasping the dishwasher handle
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):

        handle_name: str
        """
        Name of the handle to grasp
        """
        arm: str
        """
        Arm that should be used
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.grasp_dishwasher_handle().execute(self)

    def __init__(self, handle_name: str, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot grasp the dishwasher handle specified by the given parameter.

        :param handle_name: name of the handle that should be grasped
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'open'
        self.handle_name = handle_name
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for grasping motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.handle_name, self.arm)


class HalfOpeningDishwasherMotion(MotionDesignatorDescription):
    """
    Designator for half opening the dishwasher door to a given degree.
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        handle_name: str
        """
        Name of the dishwasher handle which is grasped
        """
        goal_state_half_open: float
        """
        Goal state of the door, defining the degree to open the door
        """
        arm: str
        """
        Arm that should be used
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.half_open_dishwasher().execute(self)

    def __init__(self, handle_name: str, goal_state_half_open: float, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot open the dishwasher to a given degree. This motion designator assumes that the handle
        is already grasped.

        :param handle_name: Name of the handle which is grasped
        :param goal_state_half_open: degree to which the dishwasher door should be opened
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'open'
        self.handle_name = handle_name
        self.goal_state_half_open = goal_state_half_open
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.handle_name, self.goal_state_half_open, self.arm)

class MoveArmAroundMotion(MotionDesignatorDescription):
    """
    Designator for moving the arm around the dishwasher to further open the door.
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        handle_name: str
        """
        Name of the dishwasher handle which was grasped
        """
        arm: str
        """
        Arm that should be used
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.move_arm_around_dishwasher().execute(self)

    def __init__(self, handle_name: str, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot move its arm around the dishwasher door for the next opening action. This motion designator assumes that the handle
        is not grasped anymore.

        :param handle_name: Name of the handle which was grasped
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'open'
        self.handle_name = handle_name
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for moving arm around motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.handle_name, self.arm)

class FullOpeningDishwasherMotion(MotionDesignatorDescription):
    """
    Designator for fully opening the dishwasher. Assumes that the door is already half opened and the arm is in the right position.
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        handle_name: str
        """
        Name of the dishwasher handle which was grasped
        """
        door_name: str
        """
        Name of the dishwasher door which should be opened
        """
        goal_state_full_open: float
        """
        Goal state of the door, defining the degree to open the door
        """
        arm: str
        """
        Arm that should be used
        """

        @with_tree
        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.full_open_dishwasher().execute(self)

    def __init__(self, handle_name: str, door_name: str, goal_state_full_open: float, arm: str, resolver: Optional[Callable] = None):
        """
        Lets the robot fully open the dishwasher door. This motion designator assumes that the dishwasher is already half open and the arm is in the right position.

        :param handle_name: Name of the handle, which was grasped
        :param door_name: Name of the door, which should be opened
        :param goal_state_full_open: degree to which the dishwasher door should be opened
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'open'
        self.handle_name = handle_name
        self.door_name = door_name
        self.goal_state_full_open = goal_state_full_open
        self.arm: str = arm

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.handle_name, self.door_name, self.goal_state_full_open, self.arm)

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: str
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.close().execute(self)

    def to_sql(self) -> ORMClosingMotion:
        return ORMClosingMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMClosingMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion


@dataclass
class HeadFollowMotion(BaseMotion):
    """
    Designator for moving head to human (to pose on topic /human_pose)
    """

    state: str
    """
    defines if robot should start/stop looking at humans
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.head_follow().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class TalkingMotion(BaseMotion):
    """
    Designator for talking motion, robot says a sentence.
    """

    cmd: str
    """
    Sentence what the robot should say
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.talk().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class PouringMotion(BaseMotion):
    """
    Designator for pouring
    """

    direction: str
    """
    The direction that should be used for pouring. For example, 'left' or 'right'
    """
    angle: float
    """
    the angle to move the gripper to
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.pour().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass

@dataclass
class PointingMotion(BaseMotion):
    """
    Designator for pointing to given coordinates
    Robot rotates one hand to pose
    """

<<<<<<< HEAD
    x_coordinate: float
    """
    x coordinate where the robot points to (in map frame)
    """
    y_coordinate: float
    """
    y coordinate where the robot points to (in map frame)
    """
    z_coordinate: float
    """
    z coordinate where the robot points to (in map frame)
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.pointing().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass
=======
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.objet_part, self.arm)


class TalkingMotion(MotionDesignatorDescription):
    """
    Designator for closing a container
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        cmd: str
        """
        Sentence what the robot should say
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.talk().execute(self)

    def __init__(self, cmd: str, resolver: Optional[Callable] = None):
        """
        Lets the robot close a container specified by the given parameter. This assumes that the handle is already grasped

        :param object_part: Object designator describing the handle of the drawer
        :param arm: Arm that should be used
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = cmd

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.

        :return: A resolved motion designator
        """
        return self.Motion(self.cmd)


class PouringMotion(MotionDesignatorDescription):
    """
    Designator for pouring
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        direction: str
        """
        The direction that should be used for pouring. For example, 'left' or 'right'
        """

        angle: float
        """
        the angle to move the gripper to
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.pour().execute(self)

    def __init__(self, direction: str, angle: float, resolver: Optional[Callable] = None):
        """
        Lets the robot pour based on the given parameter.
        :param direction: The direction of the pouring
        :param angle: The angle to move the gripper to
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'pour'
        self.direction: str = direction
        self.angle: float = angle

    def ground(self) -> Motion:
        """
        Default resolver for pouring motion designator, returns a resolved motion designator for the input parameters.
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.direction, self.angle)


class HeadFollowMotion(MotionDesignatorDescription):
    """
    Designator for moving head to human (to pose on topic /human_pose)
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        state: str
        """
        defines if robot should start/stop looking at humans
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.head_follow().execute(self)

    def __init__(self, state: str, resolver: Optional[Callable] = None):
        """
        Lets the robot pour based on the given parameter.
        :param state: defines start or stopping of Motion
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'headfollow'
        self.state: str = state

    def ground(self) -> Motion:
        """
        Default resolver for head following motion designator, returns a resolved motion designator for the input parameters.
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, state=self.state)


class PointingMotion(MotionDesignatorDescription):
    """
    Designator for pointing to given coordinates
    Robot rotates one hand to pose
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        x_coordinate: float
        """
        x coordinate where the robot points to (in map frame)
        """
        y_coordinate: float
        """
        y coordinate where the robot points to (in map frame)
        """
        z_coordinate: float
        """
        z coordinate where the robot points to (in map frame)
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.pointing().execute(self)

    def __init__(self, x_coordinate: float, y_coordinate: float, z_coordinate: float, resolver: Optional[Callable] = None):
        """
        Lets the robot pour based on the given parameter.
        :param x_coordinate: x coordinate where the robot points to (in map frame)
        :param y_coordinate: y coordinate where the robot points to (in map frame)
        :param z_coordinate: z coordinate where the robot points to (in map frame)
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'pointing'
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.z_coordinate = z_coordinate

    def ground(self) -> Motion:
        """
        Default resolver for pouring motion designator, returns a resolved motion designator for the input parameters.
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, self.x_coordinate, self.y_coordinate, self.z_coordinate)


class DoorOpenMotion(MotionDesignatorDescription):
    """
    Designator for opening a door
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        handle: str
        """
        name of the handle joint so that giskard knows how to open the door
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.door_opening().execute(self)

    def __init__(self, handle: str, resolver: Optional[Callable] = None):
        """
        Lets the robot pour based on the given parameter.
        :param handle: defines handle of the door to open it
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'openDoor'
        self.handle: str = handle

    def ground(self) -> Motion:
        """
        Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, handle=self.handle)


class GraspHandleMotion(MotionDesignatorDescription):
    """
    Designator for grasping a (door-)handle
    """

    @dataclasses.dataclass
    class Motion(MotionDesignatorDescription.Motion):
        handle: str
        """
        name of the handle joint so that giskard knows where to grasp the handle
        """

        def perform(self):
            pm_manager = ProcessModuleManager.get_manager()
            return pm_manager.grasp_door_handle().execute(self)

    def __init__(self, handle: str, resolver: Optional[Callable] = None):
        """
        Lets the robot pour based on the given parameter.
        :param handle: defines handle that should be grasped
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.cmd: str = 'graspDoor'
        self.handle: str = handle

    def ground(self) -> Motion:
        """
        Default resolver for grasping motion designator, returns a resolved motion designator for the input parameters.
        :return: A resolved motion designator
        """
        return self.Motion(self.cmd, handle=self.handle)

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
