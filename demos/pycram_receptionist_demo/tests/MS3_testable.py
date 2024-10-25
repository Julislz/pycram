<<<<<<< HEAD
import time

import rospy
from geometry_msgs.msg import PointStamped

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
=======
import rospy
from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
<<<<<<< HEAD

world = BulletWorld()
=======
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

world = BulletWorld("DIRECT")
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

<<<<<<< HEAD
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_8.urdf")
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = BelieveObject(names=["kitchen"])

=======
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")
giskardpy.init_giskard_interface()
# giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = BelieveObject(names=["kitchen"])
print("starting nlp")
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
<<<<<<< HEAD
=======
# giskardpy.sync_worlds()
move = PoseNavigator()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

# Declare variables for humans
host = HumanDescription("Lukas", fav_drink="Coffee")
guest1 = HumanDescription("Jessica", fav_drink="Water")
guest2 = HumanDescription("guest2")
seat_number = 2


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    response.append("None")
    callback = True


def demo_tst():
    """
    testing HRI and introduction and navigating
    """
    with real_robot:
        global callback
        global response
        test_all = False

<<<<<<< HEAD
        # HeadFollowAction(state='start').resolve().perform()
=======
        HeadFollowAction(state='start').resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

        rospy.Subscriber("nlp_out", String, data_cb)
        # desig = DetectAction(technique='attributes').resolve().perform()
        # guest1.set_attributes(desig)

<<<<<<< HEAD
        # DetectAction(technique='human', state='start').resolve().perform()
        # rospy.loginfo("human detected")

        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(3)
=======
        DetectAction(technique='human', state='start').resolve().perform()
        # rospy.loginfo("human detected")

        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(2)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        # signal to start listening
        pub_nlp.publish("start listening")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>":
            if response[1] != "<None>":
                TalkingMotion("please confirm if i got your name right").resolve().perform()
                guest1.set_drink(response[2])
                rospy.sleep(1)
                guest1.set_name(name_confirm(response[1]))

            else:
                # save heard drink
                guest1.set_drink(response[2])

                # ask for name again once
                guest1.set_name(name_repeat())

            # confirm favorite drink
            guest1.set_drink(drink_confirm(guest1.fav_drink))

        else:
            # two chances to get name and drink
            i = 0
            while i < 2:
                TalkingMotion("please repeat your name and drink loud and clear").resolve().perform()
                pub_nlp.publish("start")

                while not callback:
                    rospy.sleep(1)
                callback = False

                if response[0] == "<GUEST>":
                    guest1.set_name(response[1])
                    guest1.set_drink(response[2])
                    break
                else:
                    i += 1

        # stop looking
        TalkingMotion("i will show you the living room now").resolve().perform()
        rospy.sleep(1)
        TalkingMotion("please step out of the way and follow me").resolve().perform()
        HeadFollowAction('stop').resolve().perform()

        # stop perceiving human
        DetectAction(technique='human', state='stop').resolve().perform()

        if test_all:
            # lead human to living room
<<<<<<< HEAD
            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
=======
            #NavigateAction([pose_kitchen_to_couch]).resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
            NavigateAction([pose_couch]).resolve().perform()
            TalkingMotion("Welcome to the living room").resolve().perform()
            rospy.sleep(1)

            seat1 = DetectAction(technique='location', state='seat1').resolve().perform()
            host.set_pose(toPoseStamped(pose_red_seat[0], pose_red_seat[1], pose_red_seat[2]))
            guest1.set_pose(toPoseStamped(pose_blue_seat[0], pose_blue_seat[1], pose_blue_seat[2]))
            seat1 = seat1[1]
            if seat1.strip() == "false":
                PointingMotion(pose_blue_seat[0], pose_blue_seat[1], pose_blue_seat[2]).resolve().perform()
                HeadFollowAction('start').resolve().perform()
                pub_pose.publish(toPoseStamped(pose_red_seat[0], pose_red_seat[1], pose_red_seat[2]))
                TalkingMotion("please take a seat next to your host").resolve().perform()

            rospy.sleep(2)

        introduce(host, guest1)
<<<<<<< HEAD
        rospy.sleep(1.5)
        describe(guest1)
=======
        #rospy.sleep(1.5)
        #describe(guest1)
        TalkingMotion("end").resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


def demo_tst2():
    """
    just testing the gazing between humans -> introduce function
    """
    with real_robot:
        pub_pose = rospy.Publisher('/human_pose', PoseStamped, queue_size=10)
        TalkingMotion("Welcome to the living room").resolve().perform()
        HeadFollowAction('start').resolve().perform()
        rospy.sleep(3)

        # perceive attributes of human
        desig = DetectAction(technique='attributes').resolve().perform()
        guest1.set_attributes(desig)
        d = DetectAction(technique='human', state='start').resolve().perform()
        print(d)
        # look and point to free seat

<<<<<<< HEAD
        # pub_pose.publish(toPoseStamped(pose_red_seat[0], pose_red_seat[1], pose_red_seat[2]))
=======
        pub_pose.publish(toPoseStamped(pose_red_seat[0], pose_red_seat[1], pose_red_seat[2]))
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        # rospy.sleep(2)
        # PointingMotion(pose_blue_seat[0], pose_blue_seat[1], pose_blue_seat[2]).resolve().perform()
        TalkingMotion("please take a seat next to your host").resolve().perform()
        # HeadFollowAction('start').resolve().perform()

        # set pose of humans intern
        # host.set_pose(toPoseStamped(pose_red_seat[0], pose_red_seat[1], pose_red_seat[2]))
        # guest1.set_pose(toPoseStamped(pose_blue_seat[0], pose_blue_seat[1], pose_blue_seat[2]))
        # rospy.sleep(2)

        # introduce humans and look at them
        introduce(host, guest1)

        # describe guest one
        describe(guest1)
        rospy.sleep(3)
        TalkingMotion("end of demo").resolve().perform()
        DetectAction(technique='human', state='stop').resolve().perform()


def rotate_robot():
    """
    rotate hsr around z axis
    """
    with real_robot:
        # current robot pose
        pose1 = robot.get_pose()
        print(pose1)
        q1 = axis_angle_to_quaternion(axis=[0, 0, 1], angle=90)
        pose1.pose.orientation.x = q1[0]
        pose1.pose.orientation.y = q1[1]
        pose1.pose.orientation.z = q1[2]
        pose1.pose.orientation.w = q1[3]

        # rotating robot
        NavigateAction([pose1]).resolve().perform()


def open_tst():
    """
    just opening door
    """
    with real_robot:
        TalkingMotion("Test").resolve().perform()
        # pose1 = robot.get_pose()
        # pose2 = robot.get_complete_joint_state()

<<<<<<< HEAD
        # Pre-Pose
        pose1 = Pose([1.6, 0.2, 0], [0, 0, 1, 0])
=======
        # Pre-Pose for door opening
        pose1 = Pose([1.65, 0.52, 0], [0, 0, 1, 0])
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
        NavigateAction([pose1]).resolve().perform()
        MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
        MoveTorsoAction([0.35]).resolve().perform()

        # grasp door
        giskardpy.grasp_doorhandle("iai_kitchen/iai_kitchen:arena:door_handle_inside")
        MoveGripperMotion(motion="close", gripper="left").resolve().perform()

        # open door
        giskardpy.open_doorhandle("kitchen_2/iai_kitchen:arena:door_handle_inside")
        MoveGripperMotion(motion="open", gripper="left").resolve().perform()

<<<<<<< HEAD
        pose2 = Pose([2.2, 0.55, 0], [0, 0, 1, 0])
        NavigateAction([pose2]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
=======
        # move away from door
        pose2 = Pose([2.2, 1.0, 0], [0, 0, 1, 0])
        NavigateAction([pose2]).resolve().perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Welcome, please step in").resolve().perform()
        MoveTorsoAction([0.1]).resolve().perform()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

        TalkingMotion("end").resolve().perform()


def partesr():

    with real_robot:
        MoveTorsoAction([0.1]).resolve().perform()
        time.sleep(2)
        print(robot.links)
        print(robot.get_link_pose("hand_l_finger_tip_frame"))
        MoveGripperMotion(motion="close", gripper="left").resolve().perform()
        print(robot.get_complete_joint_state())

        print(robot.get_link_pose("hand_l_finger_tip_frame"))

        # park = ParkArmsAction([Arms.LEFT]).resolve()
        # talk = TalkingMotion("Welcome, please step in").resolve()
        # torso = MoveTorsoAction([0.1]).resolve()
        #
        # plan = park | torso
        #
        # plan.perform()

<<<<<<< HEAD

# demo_tst()
# open_tst()
partesr()

# TODO: Faliure handling
# Menschen nicht sehen
# Navigieren, bis sie Pose erreicht, neu navigieren bis Punkt erreicht
# für Platze kein freier Platz erkannt -> andere Head pose usw.
# optional: wenn Mensch vor einem wahrgenommen -> call vom Roboter, dass Mensch hinten bleibt
# Menschen usw nicht außerhalb der Arena wahrnehmen
=======
def dishwasher():
    handle_name = "sink_area_dish_washer_door_handle"
    door_name = "sink_area_dish_washer_door"
    goal_state_half_open = 0.8
    goal_state_full_open = 1.3
    arms = [Arms.LEFT]
    with real_robot:
        OpenDishwasherAction(handle_name=handle_name,
                             door_name=door_name,
                            goal_state_half_open=goal_state_half_open,
                          goal_state_full_open=goal_state_full_open,
                         arms=arms).resolve().perform()

def looking():
    with real_robot:
        talk.pub_now("start")
        print(robot.get_pose())
        move.pub_now(after_door_pose)
        talk.pub_now("rotate")
        move.pub_now(after_door_ori, interrupt_bool=False)
        talk.pub_now("navigate")
        move.pub_now(pose_corner)





looking()

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

