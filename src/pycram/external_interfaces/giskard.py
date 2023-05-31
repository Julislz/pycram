import rospy

from ..robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from ..bullet_world import BulletWorld, Object
from ..process_module import ProcessModule
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped
from giskard_msgs.msg import WorldBody, MoveResult
from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld, UpdateWorldResponse
from typing import List, Tuple, Dict

giskard_wrapper = GiskardWrapper()
giskard_update_service = rospy.ServiceProxy("/giskard/update_world", UpdateWorld)


# Believe state management between pycram and giskard


def initial_adding_objects() -> None:
    """
    Adds object that are loaded in the BulletWorld to the Giskard belief state, if they are not present at the moment.
    """
    groups = giskard_wrapper.get_group_names()
    for obj in BulletWorld.current_bullet_world.objects:
        if obj == BulletWorld.robot:
            continue
        name = obj.name + "_" + str(obj.id)
        if name not in groups:
            spawn_object(obj)


def removing_of_objects() -> None:
    """
    Removes objects that are present in the Giskard belief state but not in the BulletWorld from the Giskard belief state.
    """
    groups = giskard_wrapper.get_group_names()
    object_names = list(
        map(lambda obj: object_names.name + "_" + str(obj.id), BulletWorld.current_bullet_world.objects))
    diff = list(set(groups) - set(object_names))
    for grp in diff:
        giskard_wrapper.remove_group(grp)


def sync_worlds() -> None:
    """
    Synchronizes the BulletWorld and the Giskard belief state, this includes adding and removing objects to the Giskard
    belief state such that it matches the objects present in the BulletWorld and moving the robot to the position it is
    currently at in the BulletWorld.
    """
    bullet_object_names = set(map(lambda obj: obj.name + "_" + str(obj.id), BulletWorld.current_bullet_world.objects))
    giskard_object_names = set(giskard_wrapper.get_group_names())
    robot_name = {robot_description.i.name}
    if giskard_object_names - bullet_object_names - robot_name != set():
        giskard_wrapper.clear_world()
    initial_adding_objects()


def update_pose(object: Object) -> UpdateWorldResponse:
    """
    Sends an update message to giskard to update the object position. Might not work when working on the real robot just
    in standalone mode.

    :param object: Object that should be updated
    :return: An UpdateWorldResponse
    """
    return giskard_wrapper.update_group_pose(object.name + "_" + str(object.id),
                                             make_pose_stamped(object.get_position_and_orientation()))


def spawn_object(object: Object) -> None:
    """
    Spawns a BulletWorld Object in the giskard belief state.

    :param object: BulletWorld object that should be spawned
    """
    spawn_urdf(object.name + "_" + str(object.id), object.path, object.get_position_and_orientation())


def remove_object(object: Object) -> UpdateWorldResponse:
    """
    Removes an object from the giskard belief state.

    :param object: The BulletWorld Object that should be removed
    """
    return giskard_wrapper.remove_group(object.name + "_" + str(object.id))


def spawn_urdf(name: str, urdf_path: str, pose: Tuple[List[float], List[float]]) -> UpdateWorldResponse:
    """
    Spawns an URDF in giskard's belief state.

    :param name: Name of the URDF
    :param urdf_path: Path to the URDF file
    :param pose: Pose in which the URDF should be spawned
    :return: An UpdateWorldResponse message
    """
    urdf_string = ""
    with open(urdf_path) as f:
        urdf_string = f.read()
    pose_stamped = make_pose_stamped(pose)

    return giskard_wrapper.add_urdf(name, urdf_string, pose_stamped)


def spawn_mesh(name: str, path: str, pose: Tuple[List[float], List[float]]) -> UpdateWorldResponse:
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    pose_stamped = make_pose_stamped(pose)
    return giskard_wrapper.add_mesh(name, path, pose_stamped)


# Sending Goals to Giskard


def achieve_joint_goal(goal_poses: Dict[str, float]) -> MoveResult:
    """
    Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
    values are the goal joint positions.

    :param goal_poses:
    """
    sync_worlds()
    giskard_wrapper.set_joint_goal(goal_poses)
    return giskard_wrapper.plan_and_execute()


def achieve_cartesian_goal(goal_pose: Tuple[List[float], List[float]], tip_link: str, root_link: str) -> MoveResult:
    """
    Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
    tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    """
    sync_worlds()
    giskard_wrapper.set_cart_goal(make_pose_stamped(goal_pose), tip_link, root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_straight_cartesian_goal(goal_pose: Tuple[List[float], List[float]], tip_link: str,
                                    root_link: str) -> MoveResult:
    """
    Same as achieve_cartesian_goal but tries to move the tip_link in a straight line.
    """
    sync_worlds()
    giskard_wrapper.set_straight_cart_goal(make_pose_stamped(goal_pose), tip_link, root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> MoveResult:
    """
    Tries to move the tip_link to the position defined by goal_point using the chain defined by root_link and
    tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

    :param goal_point: The goal position of the tip_link
    :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
    :param root_link: The start link of the chain
    """
    sync_worlds()
    giskard_wrapper.set_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_straight_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> MoveResult:
    """
    Same as achieve_translation_goal but tries to move in a straight line.
    """
    sync_worlds()
    giskard_wrapper.set_straight_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_rotation_goal(quat: List[float], tip_link: str, root_link: str) -> MoveResult:
    """
    Tries to bring the tip link into the rotation defined by quat using the chain defined by root_link and
    tip_link.

    :param quat: The rotation that should be achieved, given as a quaternion
    :param tip_link: The link that should be in the rotation defined by quat
    :param root_link: The start link of the chain
    """
    sync_worlds()
    giskard_wrapper.set_rotation_goal(make_quaternion_stamped(quat), tip_link, root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_align_planes_goal(goal_normal: List[float], tip_link: str, tip_normal: List[float],
                              root_link: str) -> MoveResult:
    """
    Tries to align the plane defined by tip normal with goal_normal using the chain between root_link and
    tip_link.

    :param goal_normal: The goal plane, given as a list of XYZ
    :param tip_link: The end link of the chain that should be used.
    :param tip_normal: The plane that should be aligned with goal_normal, given as a list of XYZ
    :param root_link: The starting link of the chain that should be used.
    """
    sync_worlds()
    giskard_wrapper.set_align_planes_goal(make_vector_stamped(goal_normal), tip_link, make_vector_stamped(tip_normal),
                                          root_link)
    return giskard_wrapper.plan_and_execute()


def achieve_open_container_goal(tip_link: str, environment_link: str) -> MoveResult:
    """
    Tries to open a container in an environment, this only works if the container was added as a URDF. This goal assumes
    that the handle was already grasped. Can only handle container with 1 DOF

    :param tip_link: The End effector that should open the container
    :param environment_link: The name of the handle for this container.
    """
    sync_worlds()
    giskard_wrapper.set_open_container_goal(tip_link, environment_link)
    return giskard_wrapper.plan_and_execute()


def achieve_close_container_goal(tip_link: str, environment_link: str) -> MoveResult:
    """
    Same as achieve_open_container but closes the container this time.
    """
    sync_worlds()
    giskard_wrapper.set_close_container_goal(tip_link, environment_link)
    return giskard_wrapper.plan_and_execute()


# Managing collisions


def avoid_all_collisions() -> None:
    """
    Will avoid all collision for the next goal.
    """
    giskard_wrapper.avoid_all_collisions()


def allow_self_collision() -> None:
    """
    Will allow the robot collision with itself.
    """
    giskard_wrapper.allow_self_collision()


def avoid_collisions(object1: Object, object2: Object) -> None:
    """
    Will avoid collision between the two objects for the next goal.

    :param object1: The first BulletWorld Object
    :param object2: The second BulletWorld Object
    """
    giskard_wrapper.avoid_collision(-1, object1.name + "_" + str(object1.id), object2.name + "_" + str(object2.id))


# Creating ROS messages


def make_world_body(object: Object) -> WorldBody:
    """
    Creates a WorldBody message for a BulletWorld Object. The WorldBody will contain the URDF of the BulletWorld Object

    :param object: The BulletWorld Object
    :return: A WorldBody message for the BulletWorld Object
    """
    urdf_string = ""
    with open(object.path) as f:
        urdf_sting = f.read()
    urdf_body = WorldBody()
    urdf_body.type = WorldBody.URDF_BODY
    urdf_body.urdf = urdf_string

    return urdf_body


def make_pose_stamped(position_and_orientation: Tuple[List[float], List[float]]) -> PoseStamped:
    """
    Creates a PoseStamped message for the given position and orientation. Position is xyz in world coordinate frame and
    orientation is xyzw representing a quaternion.

    :param position_and_orientation: Tuple of position and orientation
    :return: A PoseStamped message
    """
    po, qu = position_and_orientation

    pose = PoseStamped()
    pose.header.stamp = rospy.Time().now()
    pose.header.frame_id = "map"

    pose.pose.position.x = po[0]
    pose.pose.position.y = po[1]
    pose.pose.position.z = po[2]

    pose.pose.orientation.x = qu[0]
    pose.pose.orientation.y = qu[1]
    pose.pose.orientation.z = qu[2]
    pose.pose.orientation.w = qu[3]

    return pose


def make_point_stamped(point: List[float]) -> PointStamped:
    """
    Creates a PointStamped message for the given position in world coordinate frame.

    :param point: XYZ coordinates of the point
    :return: A PointStamped message
    """
    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.point.x = point[0]
    msg.point.y = point[1]
    msg.point.z = point[2]

    return msg


def make_quaternion_stamped(quaternion: List[float]) -> QuaternionStamped:
    """
    Creates a QuaternionStamped message for the given quaternion.

    :param quaternion: The quaternion as a list of xyzw
    :return: A QuaternionStamped message
    """
    msg = QuaternionStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.quaternion.x = quaternion[0]
    msg.quaternion.y = quaternion[1]
    msg.quaternion.z = quaternion[2]
    msg.quaternion.w = quaternion[3]

    return msg


def make_vector_stamped(vector: List[float]) -> Vector3Stamped:
    """
    Creates a Vector3Stamped message, this is similar to PointStamped but represents a vector instead of a point.

    :param vector: The vector given as xyz in world frame
    :return: A Vector3Stamped message
    """
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.vector.x = vector[0]
    msg.vector.y = vector[1]
    msg.vector.z = vector[2]

    return msg
