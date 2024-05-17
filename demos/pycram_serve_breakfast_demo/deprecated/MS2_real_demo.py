from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)

target_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot_pose = robot.get_pose()

kitchen = Object("kitchen", "environment", "../../../resources/test-room.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk = Object("Milkpack", "milk", "../../../resources/milk.stl", pose=Pose([4.95, 2.66, 0.81]), color=[1, 0, 0, 1])
cereal = Object("Cerealbox", "cereal", "../../../resources/breakfast_cereal.stl", pose=Pose([4.95, 2.46, 0.83]), color=[0, 1, 0, 1])
bowl = Object("Bowl", "bowl", "../../../resources/bowl.stl", pose=Pose([4.95, 2.26, 0.765]), color=[0, 1, 0, 1])
spoon = Object("Spoon", "spoon", "../../../resources/spoon.stl", pose=Pose([4.95, 2.06, 0.74]), color=[0, 1, 0, 1])

milk_desig = BelieveObject(names=["Milkpack"])
cereal_desig = BelieveObject(names=["Cerealbox"])
bowl_desig = BelieveObject(names=["Bowl"])
spoon_desig = BelieveObject(names=["Spoon"])

# initialize and sync giskard
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")

# TODO: get pose from Knowledge
table_pose = Pose([1.5, -2.6, 0], object_orientation)

# TODO: get target pose (second table pose) from knowledge
target_pose = Pose([-0.35, -2.6, 0], target_orientation)


def move_and_detect():
    # navigate to table
    # NavigateAction(target_locations=[table_pose]).resolve().perform()
    NavigateAction(target_locations=[Pose([2.3, 0, 0])]).resolve().perform()
    NavigateAction(target_locations=[Pose([2.3, 2.4, 0], object_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([4.2, 2.4, 0], axis_angle_to_quaternion([0, 0, 1], 0))]).resolve().perform()

    # perceiving
    MoveTorsoAction([0.2]).resolve().perform()
    LookAtAction(targets=[Pose([4.95, 2.36, 0.5])]).resolve().perform()
    object_desig = DetectAction(milk_desig).resolve().perform()
    LookAtAction(targets=[Pose([4.95, 2.36, 0.7])]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    return object_desig


def try_pick_up(obj, grasps):
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except:
        print("try pick up again")
        TalkingMotion("Try pick up again")
        NavigateAction([Pose([robot_pose.position.x - 0.3, robot_pose.position.y, robot_pose.position.z],
                             robot_pose.orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()
        # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
        new_object = object_desig[obj.name]
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()
        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} object on the table? Thanks")
            TalkingMotion(f"Please push down my hand, when I can grab the {obj.name}.")


def sort_obj_list(obj_dict):
    wished_sorted_obj_list = ["Bowl", "Milkpack", "Cerealbox", "Spoon"]
    obj_list = []
    sorted_obj_list = []
    for key, value in obj_dict.items():
        if key in wished_sorted_obj_list:
            obj_list.append(value)

    if len(obj_list) != len(sorted_obj_list):
        TalkingMotion("Could not perceive all needed objects")

    # print list
    j = 0
    helper = []
    for object in obj_list:
        helper.append(object.name)
    print("helper: ", helper)
    for i in range(0, len(wished_sorted_obj_list)):
        print("index: ", i)
        if wished_sorted_obj_list[i] in helper:
            sorted_obj_list.insert(j, obj_list[helper.index(wished_sorted_obj_list[i])])
            j = j + 1

    for obj in sorted_obj_list:
        print("Objects in list: ", obj.name)

    return sorted_obj_list


def switch(obj):
    z = 0
    if obj.name == "MetalMug":
        z = 0.84
    return z


with semi_real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    object_list = move_and_detect()
    sorted_list = sort_obj_list(object_list)

    # pick up and drop all five objects
    for obj in sorted_list:
        # use robokudo to perceive object pose
        # object_desig = DetectAction(object_list[index]).resolve().perform()
        new_pose = obj.pose
        print(new_pose)
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([4.2, new_pose.position.y, 0])]).resolve().perform()
        # determine grasping movement between objects
        if obj.name == "Spoon" or obj.name == "Bowl":
            try_pick_up(obj, "top")
            # PickUpAction(object_designator_description=object,
            #              arms=["left"],
            #              grasps=["top"]).resolve().perform()
            print("grasp from top")
        else:
            try_pick_up(obj, "front")
            # PickUpAction(object_designator_description=object,
            #              arms=["left"],
            #              grasps=["front"]).resolve().perform()
            print("grasp from front")

        # turn around and navigate to target
        NavigateAction(target_locations=[Pose([4, new_pose.position.y, 0], target_orientation)]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.4, new_pose.position.y, 0])]).resolve().perform()

        # place the object


        place_pose = Pose([-0.88, new_pose.position.y, new_pose.position.z])
        PlaceAction(obj, [place_pose], ["left"]).resolve().perform()
        print("placing finished")
        ParkArmsAction([Arms.LEFT]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([0, -2.6, 0], object_orientation)]).resolve().perform()
    print("finished")
