from pycram.designators.action_designator import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_storing_groceries_demo.utils.misc import sort_obj

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "couch-whole_kitchen2.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])


with real_robot:
    shelf_obj = DetectAction(technique='all').resolve().perform()
    sorted_obj_list = sort_obj(shelf_obj, robot)

    # navigate to pp.py area
    NavigateAction([pose_test_area]).resolve().perform()

    # optional goal: open cabinet door
    # idea: perceiving cabinet once in the beginning and classifying shelfs?
    door_handle_desig = ObjectPart(names=["cabinet_handle"], part_of=kitchen_desig.resolve())

    # position hsr in front of door for opening
    pre_pose = Pose()
    NavigateAction([pre_pose]).resolve().perform()
    OpenAction(object_designator_description=door_handle_desig, arms=["left"]).resolve().perform()

    # loop so that robots stores 5 objects
    for i in range(5):

        # navigate to table and perceive objects
        found_obj = navigate_and_detect("table")
        if not found_obj:
            # try perceiving again
            found_obj = DetectAction(technique='all').resolve().perform()
            giskardpy.sync_worlds()

            if not found_obj:
                TalkingMotion("no object found").resolve().perform()

        # Knowledge call which object to pick up
        # possible criteria closest object, easiest to pick up, easiest to classify
        # coordinate with knowledge and perception which objects to prioritize
        sorted_obj_list = sort_obj(found_obj)
        obj = knowrob.get_obj_to_pick(sorted_obj_list)

        # pick up chosen object and navigate to cabinet
        try:
            # position robot accuratly before picking up necessary?
            pre_pose = Pose()
            NavigateAction([pre_pose]).resolve().perform()
            PickUpAction(obj, ["left"], ["front"]).resolve().perform()
        except (EnvironmentUnreachable, GripperClosedCompletely):
            # failure handling
            # or just try to pick up again like in celinas/mohammeds demo
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion(f"please give me the {obj.type} on the table?").resolve().perform()
            MoveGripperMotion("open", "left").resolve().perform()
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()

        # navigate to cabinet and detect groups in shelf
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # shelf areas get classified new for every object, maybe once at the beginning is enough
        shelf_obj = navigate_and_detect("cabinet")

        # knowledge call with sorted list and picked up object
        # knowledge returns pose to place object (or is that a job of planning?)
        sorted_obj_list = sort_obj(shelf_obj)
        place_pose = knowrob.get_self_to_place(sorted_obj_list, obj)

        # place object in shelf with objects of same category
        try:
            PlaceAction(obj, ["left"], ["front"], [place_pose]).resolve().perform()
        except:
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("please place the object for me. i will let go of object").resolve().perform()
            time.sleep(4)
            MoveGripperMotion("open", "left").resolve().perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()

    # end of demo
    TalkingMotion("finished placing objects").resolve().perform()
