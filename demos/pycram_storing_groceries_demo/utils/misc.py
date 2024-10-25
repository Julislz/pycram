<<<<<<< HEAD
=======
from collections import Counter

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
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

<<<<<<< HEAD
=======
# dict to categories objects
categories = {
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Cutlery": ["Fork", "Spoon", "Knife", "Metalplate", "Metalbowl"],
    "Drinkware": ["Pitcher", "Wineglass", "Metalmug", "Cupblue", "Cupgreen", "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner",
                          "Dishwashertab"],
    "Tools": ["Scissors", "Screwdriver", "Clamp", "Hammer", "Wooden_block"],
    "Toys/Games": ["Rubikscube"],
    "Other": ["Largemarker"],
    "Snacks": ["Crackerbox", "Pringleschipscan"],
    "Breakfast Foods": ["Cerealbox", "Mueslibox", "Cornybox"],
    "Canned Goods": ["Tomatosoupcan", "Tunafishcan", "Pottedmeatcan", "Masterchefcan"],
    "Beverage Ingredients": ["Coffeecan", "Coffeepack", "Sugarbox", "Milkpackja"],
    "Condiments": ["Mustardbottle"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"]
}

# make object name to key for faster access to category
item_to_category = {item: category for category, items in categories.items() for item in items}

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

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
<<<<<<< HEAD
        #NavigateAction([pose_table]).resolve().perform()
=======
        # NavigateAction([pose_table]).resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

    TalkingMotion("Perceiving").resolve().perform()

    # von Celina und mohammed
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


<<<<<<< HEAD
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

=======
def sort_new(perceived: dict, robot: BulletWorldObject):
    """
    function that sorts the perceived objects from perception.
    returns a list with two lists.
    the first entry is a sorted list of the objects. It is sorted by distance to robot
    the second entry is a dict. object name is the key and the value is the pose of the object
    :param perceived: data from perception
    :param robot: robot model in pycram to get current pose
    """
    obj_list = {}
    sorted_obj_list = []

    obj_dict = perceived[1]
    robot_pose = robot.get_pose()
    for value in obj_dict:
        # calculate distance
        distance = math.sqrt(pow((obj_dict[value].pose.position.x - robot_pose.pose.position.x), 2) +
                             pow((obj_dict[value].pose.position.y - robot_pose.pose.position.y), 2) +
                             pow((obj_dict[value].pose.position.z - robot_pose.pose.position.z), 2))

        # get position of object
        obj_pose = Pose()
        obj_pose.position = [obj_dict[value].pose.position.x, obj_dict[value].pose.position.y,
                             obj_dict[value].pose.position.z]
        obj_pose.orientation = [obj_dict[value].pose.orientation.x, obj_dict[value].pose.orientation.y,
                                obj_dict[value].pose.orientation.z, obj_dict[value].pose.orientation.w]

        # put everything in a dict
        key = obj_dict[value].type + "0"
        i = 1
        while key in obj_list:
            key = obj_dict[value].type + str(i)
        obj_list[key] = obj_pose
        sorted_obj_list.append((obj_dict[value].type, distance))

    sorted_obj_list = sorted(sorted_obj_list, key=lambda distance: distance[1])

    return [sorted_obj_list, obj_list]


def sort_and_remove_duplicates(unsorted_list):
    """
    function that sorts a list of strings by frequency and removes duplicates
    not case-sensitive
    :param unsorted_list: list with strings to sort
    """
    # letters to lowercase
    unsorted_list = [s.lower() for s in unsorted_list]

    # count frequency of each string
    frequency = Counter(unsorted_list)

    # remove duplicates (transform to set and back to list)
    unique_strings = list(set(unsorted_list))

    # sort list according to frequency
    sorted_unique_strings = sorted(unique_strings, key=lambda x: (-frequency[x], x))

    return sorted_unique_strings


def order_categorys(objects: List):
    """
    function that makes a frequency ordered list of category's given a list of objects
    the first entry of the returned list is the category that fits the most objects of the given list
    e.g. ["apple","banana","fork"] -> ["fruit", "cutlery"]
    :param objects: list of objects that get a category assigned
    """
    categorys = []
    for obj in objects:
        try:
            categorys.append(item_to_category[obj][:-1])
        except KeyError:
            categorys.append("unknown")

    ordered_categorys = sort_and_remove_duplicates(categorys)
    return ordered_categorys


def get_category(obj: str):
    """
    function that returns fitting category for object
    or unknown when object is not in dictionary
    :param obj: name of object to classify
    """
    try:
        return item_to_category[obj][:-1]
    except KeyError:
        return "unknown"


def find_nearest(obj_info: List[float], shelves: List[ShelfCompartmentDescription]) -> ShelfCompartmentDescription:
    """
    function to assign a shelf compartment from List to an object. works by choosing nearest compartment to object
    using z-coordinate distances
    :param obj_info: [z-coord/obj_height, x/y-coord of object] -> information from
    object that is needed to assign compartment and update shelf areas
    :param shelves: list of shelves that the object can be assigned to and that get updated
    """
    distance = 1000
    fitting_shelf = shelves[0]
    for shelf in shelves:
        new_distance = abs(obj_info[0] - shelf.height)
        if new_distance < distance:
            distance = new_distance
            fitting_shelf = shelf

    index = 0
    for area in fitting_shelf.placing_areas:
        if area[0] <= obj_info[1] <= area[1]:
            fitting_shelf.set_area_occupied(index, True)
            break
        index += 1
    return fitting_shelf


def order_items_to_shelves(obj_list: List, shelves: List[ShelfCompartmentDescription]):
    """
    function to update shelf compartments with perceived information
    every shelf compartment gets category list depending on which objects were perceived on compartment
    and the occupied areas are updated
    IMPORTANT: changing variable depends on how the shelf is oriented (either x or y is different for objects)
    -> needs to be changed according to map/arena

    :param obj_list: list of perceived objects that need to be assigned to a shelf
    :param shelves: list of shelves that the objects get assigned to
    """
    for object in obj_list:

        obj_pose = obj_list[object].pose
        # height of object to determine shelf fitting compartment
        height = obj_pose.position.z

        # y or x pose to determine in which area (right, middle, left) the object is standing
        changing_var_pose = obj_pose.position.y

        # object gets assigned a compartment and the compartment areas are updated
        fitting_shelf = find_nearest([height, changing_var_pose], shelves)
        # store category's in shelf object
        fitting_shelf.category.append(get_category(object))

    for shelf in shelves:
        # sort all stored category's by frequency and remove duplicates
        shelf.category = sort_and_remove_duplicates(shelf.category)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


