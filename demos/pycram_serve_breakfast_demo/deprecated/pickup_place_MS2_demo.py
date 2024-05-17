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

target_orientation = axis_angle_to_quaternion([0, 0, 1], 270)

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()

kitchen = Object("kitchen", "environment", "../../../resources/test-room.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk = Object("Milkpack", "milk", "../../../resources/milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])
cereal = Object("Cerealbox", "cereal", "../../../resources/breakfast_cereal.stl", pose=Pose([-2.5, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
metalmug = Object("Metalmug", "metalmug", "../../../resources/bowl.stl", pose=Pose([-2.9, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
cheezels = Object("Crackerbox", "cheezels", "../../../resources/breakfast_cereal.stl", pose=Pose([-2.3, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
pringles = Object("Pringleschipscan", "pringles", "../../../resources/milk.stl", pose=Pose([-3.1, 2.3, 0.43]), color=[0, 1, 0, 1])


milk_desig = BelieveObject(names=["Milkpack"])
cereal_desig = BelieveObject(names=["Cerealbox"])
metalmug_desig = BelieveObject(names=["Metalmug"])
cheezels_desig = BelieveObject(names=["Crackerbox"])
pringles_desig = BelieveObject(names=["Pringleschipscan"])

# initialize and sync giskard
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")

#TODO: get pose from Knowledge
table_pose = Pose([-2.6, 1.5, 0], object_orientation)

#TODO: get target pose (second table pose) from knowledge
target_pose = Pose([-2.6, -0.35, 0], target_orientation)

def move_and_detect():
    # navigate to table
    #NavigateAction(target_locations=[table_pose]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    # perceiving
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.5])]).resolve().perform()
    object_desig = DetectAction(milk_desig, technique='all').resolve().perform()
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.7])]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


    return object_desig

def sort_obj_list(obj_dict):
    wished_sorted_obj_list = ["Bowl", "Metalmug", "Fork", "Spoon", "Plate"]
    obj_list = []
    sorted_obj_list = []
    for key, value in obj_dict.items():
        if key in wished_sorted_obj_list:
            obj_list.append(value)

    if len(obj_list) != len(sorted_obj_list):
        TalkingMotion("Could not perceive all needed objects")

    #print list
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

with semi_real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    object_list = move_and_detect()
    sorted_list = sort_obj_list(object_list)

    # pick up and drop all five objects
    for object in sorted_list:
        # use robokudo to perceive object pose
       # object_desig = DetectAction(object_list[index]).resolve().perform()
        new_pose = object.pose
        print(new_pose)
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([new_pose.position.x, 1.5, 0], object_orientation)]).resolve().perform()
        # determine grasping movement between objects
        if object.name == "Spoon" or object.name == "Bowl":
            PickUpAction(object_designator_description=object,
                         arms=["left"],
                         grasps=["top"]).resolve().perform()
            print("grasp from top")
        else:
            PickUpAction(object_designator_description=object,
                         arms=["left"],
                         grasps=["front"]).resolve().perform()
            print("grasp from front")

        # turn around and navigate to target
        NavigateAction(target_locations=[Pose([new_pose.position.x, 1.5, 0], target_orientation)]).resolve().perform()
        NavigateAction(target_locations=[Pose([new_pose.position.x, 0.5, 0])]).resolve().perform()

        #place the object
        place_pose = Pose([new_pose.position.x, -0.35, new_pose.position.z])
        PlaceAction(object, [place_pose], ["left"]).resolve().perform()
        print("placing finished")
        ParkArmsAction([Arms.LEFT]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    print("finished")
