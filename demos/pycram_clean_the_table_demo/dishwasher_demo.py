import time
from enum import Enum
import rospy.core
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
# from pycram.external_interfaces.knowrob import get_table_pose
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

# Create an instance of the StartSignalWaiter
#start_signal_waiter = StartSignalWaiter()
text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

#Dishwashertab hinzufügen
# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 4.84

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
dishwasher_main_name = "sink_area_dish_washer_main"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()


# Wait for the start signal
#start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")


class PlacingXPose(Enum):
    """
    Differentiate the x pose for placing
    """
    CUTLERY = 2.376
    SPOON = 2.376
    FORK = 2.376
    PLASTICKNIFE = 2.376
    KNIFE = 2.376
    METALBOWL = 2.83
    METALMUG = 2.79
    METALPLATE = 2.81


class PlacingYPose(Enum):
    """
    Differentiate the y pose for placing
    """
    CUTLERY = -1.59
    SPOON = -1.59
    FORK = -1.59
    PLASTICKNIFE = -1.59
    KNIFE = -1.59
    METALBOWL = -1.73
    METALMUG = -1.75
    METALPLATE = -1.65 # 1.65


def get_pos(obj_type: str):
    """
      Getter for x and y value for placing the given object type.

      :param obj_type: type of object, which x and y pose for placing we want
      :return: the tupel of x and y value for placing that object
      """
    x_val = PlacingXPose[obj_type.upper()].value
    y_val = PlacingYPose[obj_type.upper()].value
    return x_val, y_val


def navigate_to(location_name: str, y: Optional[float] = None):
    """
    Navigates to the couch table or to the dishwasher on different sides.

    :param y: y pose to navigate to the couch table for picking up objects
    :param location_name: defines the name of the location to move to
    """
    if location_name == "couch table" and y is not None:
        NavigateAction(target_locations=[Pose(move_to_the_middle_table_pose, [0, 0, 1, 1])]).resolve().perform()
        NavigateAction(target_locations=[Pose([3.9, y, 0], [0, 0, 0, 1])]).resolve().perform()
    elif location_name == "dishwasher_left":
        NavigateAction(target_locations=[Pose(move_to_the_middle_dishwasher_pose, [0, 0, -1, 1])]).resolve().perform()
        NavigateAction(target_locations=[Pose([3.45, -1.34, 0], [0, 0, 1, 0])]).resolve().perform()
    elif location_name == "dishwasher_right":
        NavigateAction(target_locations=[Pose(move_to_the_middle_dishwasher_pose, [0, 0, -1, 1])]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.95, -1.48, 0], [0, 0, 0, 1])]).resolve().perform()
    elif location_name == "dishwasher":
        NavigateAction(target_locations=[Pose([2.55, -0.9, 0], [0, 0, -1, 1])]).resolve().perform()
    else:
        rospy.logerr("Failure. Y-Value must be set for the navigateAction to the couch table")


# Main interaction sequence with real robot
with ((real_robot)):

    rospy.loginfo("Starting demo")
    text_to_speech_publisher.pub_now("Starting demo")

    navigate_to("dishwasher")

    # Dishwasher opening
    MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
    image_switch_publisher.pub_now(2)
    OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, ["left"]).resolve().perform()

    text_to_speech_publisher.pub_now("Please pull out the lower rack")

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion("open", "left").resolve().perform()

    #MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

    # Giving the plate
    text_to_speech_publisher.pub_now("Can you please give me the plate.")
    image_switch_publisher.pub_now(5)
    MoveGripperMotion("open", "left").resolve().perform()
    time.sleep(3)

    text_to_speech_publisher.pub_now("Grasping.")

    MoveGripperMotion("close", "left").resolve().perform()
    z = 0.55

    # Moves arm to the side
    MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()
    x_y_pos = get_pos("Metalplate")

    x_pos = x_y_pos[0]
    y_pos = x_y_pos[1]
    # print(f"x: {x_pos}")
    # print(f"y: {y_pos}")

    navigate_to("dishwasher_left")

    # Placing
    text_to_speech_publisher.pub_now("Placing")
    image_switch_publisher.pub_now(8)
    grasp = "front"

    PlaceGivenObjAction(["Metalplate"], ["left"],
                        [Pose([x_pos, y_pos, z])], [grasp], False).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # # detect objects
    # object_desig = navigate_and_detect()
    # image_switch_publisher.publish_image_switch(0)
    #
    # # sort objects based on distance and which we like to keep
    # sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)
    #
    # #picking up and placing objects
    # pickup_and_place_objects(sorted_obj)
    #
    # new_obj_desig = failure_handling1(sorted_obj)
    # failure_handling2(sorted_obj, new_obj_desig)

    rospy.loginfo("Done!")
    text_to_speech_publisher.pub_now("Done")
    image_switch_publisher.pub_now(3)