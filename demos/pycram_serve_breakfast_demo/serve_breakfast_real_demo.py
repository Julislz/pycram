from enum import Enum
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from demos.pycram_hsrb_real_test_demos.utils.misc import try_pick_up

# from pycram.external_interfaces.knowrob import get_table_pose
# TODO: im Allgemeinen müssen die Werte beim Navigieren und detecten angepasst werden
# TODO: sortobjects, getbowl, get_free_spaces testen
# TODO: viellericht bowl_exists, place_pose_exists, table_pose entfernen

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalbowl", "mueslibox", "Milkpackja"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the popcorn table
# TODO: Wert anpassen
table_pose = 1.04

# the bowl to pour in
bowl = None

# if we already have places to place or not
place_pose_exists = False

# if bowl perceived or not
bowl_exists = False

# free places for placing
place_pose = None

# list of sorted placing poses
sorted_places = []

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_door_open_9.urdf")

giskardpy.init_giskard_interface()

giskardpy.sync_worlds()


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 0.8
    SPOON = 0.8
    FORK = 0.8
    PLASTICKNIFE = 0.8
    KNIFE = 0.8
    METALBOWL = 0.84
    MILKPACK = 0.88
    METALMUG = 0.8
    CEREALBOX = 0.9
    METALPLATE = 0.9  # table height plus radius of plate


def try_detect(pose: Pose, location: bool):
    """
    Navigates to the popcorn table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    LookAtAction(targets=[pose]).resolve().perform()
    TalkingMotion("Perceiving").resolve().perform()
    try:
        if location:
            object_desig = DetectAction(technique='location', state='long_table').resolve().perform()
        else:
            object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to the popcorn table or to the table on the other side.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "shelf":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    elif table_name == "long table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0, 1])]).resolve().perform()


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global table_pose, CUTLERY, bowl, place_pose_exists, bowl_exists, place_pose

    for value in range(len(sorted_obj)):
        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["mueslibox", "Cerealbox", "Crackerbox"]:
            sorted_obj[value].type = "Cerealbox"

        if sorted_obj[value].type in ["Milkpackja"]:
            sorted_obj[value].type = "Milkpack"

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"
            if sorted_obj[value].pose.position.z >= 0.65:
                sorted_obj[value].pose.position.z = 0.72
            elif sorted_obj[value].pose.position.z >= 0.4:
                sorted_obj[value].pose.position.z = 0.46
            else:
                sorted_obj[value].pose.position.z = 0.07

            # TODO: muss noch getestet und angepasst werden
            if sorted_obj[value].type in CUTLERY:
                sorted_obj[value].pose.position.y = 3.25

        TalkingMotion("Picking up with: " + grasp).resolve().perform()
        try_pick_up(robot, sorted_obj[value], grasp)

        # move back a little bit
        # TODO: vielleicht das Abziehen von y-pose ändern
        navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.25, "shelf")

        place_objects(True, sorted_obj, value, grasp)


def place_objects(first_placing, objects_list, index, grasp):
    global bowl, place_pose_exists, bowl_exists, place_pose, sorted_places
    i = 0

    if first_placing:
        object_type = objects_list[index].type
    else:
        object_type = objects_list[index]

    # placing the object
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").resolve().perform()
    # MoveGripperMotion("open", "left").resolve().perform()
    navigate_to(2.1, 1.9, "long table")
    navigate_to(3.65, 2.2, "long table")

    # erste Variante
    ###############################################################################
    if object_type is not "Cutlery":
        place_poses_list = try_detect(Pose([5.1, 2.1, 0.21], [0, 0, 0, 1]), True)
        sorted_places = get_free_spaces(place_poses_list[1])

    if object_type is not "Metalbowl":
        object_desig = try_detect(Pose([5.1, 2.1, 0.21], [0, 0, 0, 1]), False)
        bowl = get_bowl(object_desig)

    # choose place pose
    if object_type is "Metalbowl" and len(sorted_places) == 3:
        place_pose = sorted_places[1]
    else:
        place_pose = sorted_places[0]

    # zweite Variante
    ################################################################################
    # # TODO: die bessere Variante vom detecten auswählen
    # if not place_pose_exists or not bowl_exists:
    #     if not place_pose_exists:
    #         # detect free space to place
    #         place_poses_list = try_detect(Pose([5.1, 2.1, 0.21], [0, 0, 0, 1]), True)
    #         sorted_places = get_free_spaces(place_poses_list[1])
    #         place_pose_exists = True
    #     else:
    #         if object_type in ["Cerealbox", "Milkpack", "Cutlery"]:
    #             if not bowl_exists:
    #                 object_desig = try_detect(Pose([5.1, 2.1, 0.21], [0, 0, 0, 1]), False)
    #                 bowl = get_bowl(object_desig)
    #                 if bowl is not None:
    #                     bowl_exists = True
    #
    # # choose place pose
    # if object_type is "Metalbowl" and len(sorted_places) == 3:
    #     place_pose = sorted_places[1]
    # else:
    #     place_pose = sorted_places[0]
    # place_pose = sorted_places[i]
    ##################################################################################

    if object_type in ["Cerealbox", "Milkpack", "Cutlery"]:
        # TODO: vielleicht dieses "if" entfernen
        if bowl is None:
            # Failure handling

            # ein Schritt nach rechts, links oder hinten :)
            navigate_to(robot.get_pose().pose.position.x - 0.1, robot.get_pose().pose.position.y, "long table")
            new_object_deign = try_detect(Pose([5.1, 2.1, 0.21], [0, 0, 0, 1]), False)
            bowl = get_bowl(new_object_deign)
            ########################################
            # oder die letzte place_pose nehmen (die Pose vom bowl)
            # TODO: richtig anpassen und vielleicht entfernen
            # if bowl is None:
            # bowl = place_pose

        if bowl is not None:
            if object_type in ["Cerealbox", "Milkpack"]:
                # TODO: Werte anpassen
                navigate_to(3.8, bowl.pose.position.y, "long table")
                if robot.get_pose().pose.position.y <= bowl.pose.position.y:
                    PouringAction([bowl.pose], ["left"], ["left"], [-1.6]).resolve().perform()
                else:
                    PouringAction([bowl.pose], ["left"], ["right"], [1.6]).resolve().perform()
            else:
                place_pose.pose.position.x = bowl.pose.position.x
                place_pose.pose.position.y = bowl.pose.position.y - 0.2
                place_pose.pose.position.x = bowl.pose.position.z

    # TODO: Werte anpassen
    navigate_to(3.8, place_pose.pose.position.y, "long table")
    TalkingMotion("Placing").resolve().perform()
    z = get_z(object_type)
    if first_placing:
        PlaceAction(objects_list[index], ["left"], [grasp],
                    [Pose([place_pose.pose.position.x, place_pose.pose.position.y, z])]).resolve().perform()
    else:
        PlaceGivenObjAction([objects_list[index]], ["left"],
                            [Pose([place_pose.pose.position.x, place_pose.pose.position.y, z])],
                            [grasp]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # TODO: im Falle des Entfernen der zweiten Variante dieses "if entfernen mit i oben auch"
    # index of next place pose
    if object_type in ["Cerealbox", "Milkpack", "Metalbowl"]:
        i += 1

    # navigates back if a next object exists
    if index + 1 < len(objects_list):
        TalkingMotion("Navigating").resolve().perform()
        # TODO: Werte anpassen
        if first_placing:
            navigate_to(sorted_obj[index + 1].pose.position.x, 1.87, "shelf")
        else:
            navigate_to(2.1, 1.87, "shelf")


def get_z(obj_type: str):
    """
    Getter for z value for placing the given object type.

    :param obj_type: type of object, which z pose we want
    :return: the int z value for placing that object
    """
    return PlacingZPose[obj_type.upper()].value

def remove_objects(value):
    # remove all objects that were seen and transported so far
    if value.type in wished_sorted_obj_list:
        wished_sorted_obj_list.remove(value.type)
    # if the type is cutlery, the real type is not easily reproducible.
    # remove one cutlery object of the list with the highest chance that it was found and transported.
    if value.type == "Cutlery":
        if "Spoon" in wished_sorted_obj_list:
            print("deleted fork")
            wished_sorted_obj_list.remove("Fork")
        elif "Fork" in wished_sorted_obj_list:
            print("deleted spoon")
            wished_sorted_obj_list.remove("Spoon")
        elif "Plasticknife" in wished_sorted_obj_list:
            print("deleted knife")
            wished_sorted_obj_list.remove("Plasticknife")

    if value.type == "Cearealbox":
        if "mueslibox" in wished_sorted_obj_list:
            print("deleted mueslibox")
            wished_sorted_obj_list.remove("mueslibox")
        elif "Cerealbox" in wished_sorted_obj_list:
            print("deleted Cerealbox")
            wished_sorted_obj_list.remove("Cerealbox")
        elif "Crackerbox" in wished_sorted_obj_list:
            print("deleted Crackerbox")
            wished_sorted_obj_list.remove("Crackerbox")

    if value.type == "Milkpack":
        if "Milkpackja" in wished_sorted_obj_list:
            print("deleted Milkpackja")
            wished_sorted_obj_list.remove("Milkpackja")



with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    # TODO: vielleicht entfernen in der finalen Demo
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").resolve().perform()
    # navigate to door
    # TODO: koordinaten bestimmen
    # NavigateAction(target_locations=[Pose([0, 0, 0], [0, 0, 0, 1])]).resolve().perform()

    # navigate to shelf
    navigate_to(2.1, 1.87, "shelf")
    obj_desig = try_detect(Pose([2.25, 3.25, 0.21], [0, 0, 0.7, 0.7]), False)
    sorted_obj = sort_objects(obj_desig, wished_sorted_obj_list)
    pickup_and_place_objects(sorted_obj)

    # failure handling part 1
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive and pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            remove_objects(value)

        TalkingMotion("Navigating").resolve().perform()
        navigate_to(2.1, 1.87, "shelf")
        new_object_desig = try_detect(Pose([2.25, 3.25, 0.21], [0, 0, 0.7, 0.7]), False)
        new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
        pickup_and_place_objects(new_sorted_obj)

        # failure handling part 2
        final_sorted_obj = sorted_obj + new_sorted_obj
        if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            navigate_to(2.1, 1.87, "shelf")
            print("second Check")

            for value in final_sorted_obj:
                # remove all objects that were seen and transported so far
                remove_objects(value)

            for val in range(len(wished_sorted_obj_list)):
                grasp = "front"
                if wished_sorted_obj_list[val] in (["Metalbowl"] + CUTLERY):
                    grasp = "top"

                print(f"next object is: {wished_sorted_obj_list[val]}")
                TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
                              f"on the table?").resolve().perform()
                time.sleep(4)
                MoveGripperMotion("close", "left").resolve().perform()

                place_objects(False, wished_sorted_obj_list, val, grasp)

    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()

########################### PSEUDO-CODE ################################
# if door open then enter room
# navigate to shelf
# perceive
# sort objects in a list like this (bowl , cereal, milk, spoon)
# loop: pickup object
####### navigate to table
####### if object milk or cereal or spoon -> perceive bowl
#######  if milk or cereal -> pour into bowl
#######   if spoon place object right of bowl, ELSE: place on free space
#######
