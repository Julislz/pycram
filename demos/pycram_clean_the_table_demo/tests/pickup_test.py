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
#image_switch_publisher = ImageSwitchPublisher()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

#Dishwashertab hinzufügen
# Wished objects for the Demo
wished_sorted_obj_list = ["Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 5.69

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
dishwasher_main_name = "sink_area_dish_washer_main"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])
# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()



# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])



# giskardpy.initial_adding_objects()
# giskardpy.sync_worlds()


# Wait for the start signal
#start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
#rospy.loginfo("Start signal received, now proceeding with tasks.")
# code 810 Fehlercode fürs placing
with real_robot:

    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    #text_to_speech_publisher.pub_now("Can you please give me the plate.")

    # MoveGripperMotion("open", "left").resolve().perform()
    # time.sleep(3)
    #
    # text_to_speech_publisher.pub_now("Grasping.")
    #
    # MoveGripperMotion("close", "left").resolve().perform()
    # pose = Pose([1.58,5.79, 0.73],[0,0,1,1])
    # text_to_speech_publisher.pub_now("Placing.")
    #
    #
    #
    # giskardpy.giskard_wrapper.monitor_placing_in_old(goal_pose=pose, align="", grasp="", threshold_name="Placing", object_type="Standard")
    # giskardpy.giskard_wrapper.execute()


    object_desig = DetectAction(technique='all').resolve().perform()
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)
    grasp = "front"
    print(sorted_obj[0])
   # sorted_obj[0].type = "Cutlery"
    if sorted_obj[0].type == "Cutlery" and sorted_obj[0].pose.position.y > table_pose + 0.115:
        print("in here!!!!!")
        sorted_obj[0].pose.position.y -= 0.11
    text_to_speech_publisher.pub_now("Pick up")
    try_pick_up(robot, sorted_obj[0], grasp)
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # NavigateAction(target_locations=[Pose([2.3, 5.22, 0], [0, 0, 1, 1])]).resolve().perform()
