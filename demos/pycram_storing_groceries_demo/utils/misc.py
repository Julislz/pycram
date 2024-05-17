from demos.pycram_storing_groceries_demo.utils.misc import *
from docutils.nodes import math
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *

# poses declared before the challenge
pose_test_area = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_cabinet = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_table = Pose([1.5, 2.0, 0], [0, 0, 1, 0])


def navigate_and_detect(location: str):
    """
    Navigates to given location and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    if location == "cabinet":
        NavigateAction([pose_cabinet]).resolve().perform()
    elif location == "table":
        TalkingMotion("done").resolve().perform()
        #NavigateAction([pose_table]).resolve().perform()

    TalkingMotion("Perceiving").resolve().perform()

    # von Celina und mohammed
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def sort_obj(obj_dict: dict, robot: BulletWorldObject):
    """
    sort objects perceived so that knowledge can work with list

    :return: list for knowledge call
    """

    sorted_objects = []
    obj_list = []

    if len(obj_dict) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    first, *remaining = obj_dict
    robot_pose = robot.get_pose()
    # calculate euclidian distance for all found object in a dictionary
    for dictionary in remaining:
        for value in dictionary.values():
            distance = math.sqrt(pow((value.pose.position.x - robot_pose.pose.position.x), 2) +
                                 pow((value.pose.position.y - robot_pose.pose.position.y), 2) +
                                 pow((value.pose.position.z - robot_pose.pose.position.z), 2))

            # fill dict with objects and their distance.
            # Add seen objects only once, if seen multiple times
            if value.type not in obj_list:
                obj_list.append((value, distance))

    # sort all objects that where in the obj_dict
    sorted_objects = sorted(obj_list, key=lambda distance: distance[1])

    return [a for (a, _) in sorted_objects]



