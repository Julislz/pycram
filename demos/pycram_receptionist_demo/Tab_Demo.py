from pycram.designators.action_designator import DetectAction, NavigateAction
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False

# Declare variables for humans
host = HumanDescription("Yannis", fav_drink="ice tea")
guest1 = HumanDescription("guest1")
guest2 = HumanDescription("guest2")
seat_number = 2


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    response.append("None")
    callback = True


def describe_human():
    """
    testing HRI and introduction and navigating
    """
    with real_robot:

        giskardpy.move_head_to_human()
        while True:
            desig = DetectAction(technique='attributes').resolve().perform()

            # extract information from query
            gender = desig[1].res[0].attribute[0][13:19]
            if gender[0] != 'f':
                gender = gender[:4]
            print("gender: " + str(gender))
            clothes = desig[1].res[0].attribute[2][20:]
            brightness_clothes = desig[1].res[0].attribute[1][5:]
            hat = desig[1].res[0].attribute[3][20:]

            TalkingMotion("Hello, i am Toya and i learned to describe humans better this week").resolve().perform()
            DetectAction(technique='human', state='start').resolve().perform()

            TalkingMotion("let me show you. I will try to describe you").resolve().perform()

            # gender, headgear, clothing, brightness of clothes
            guest1.set_attributes([gender, hat, clothes, brightness_clothes])
            rospy.sleep(2)

            describe(guest1)
            rospy.sleep(2)

            TalkingMotion("i hope that was accurate").resolve().perform()
            rospy.sleep(1)

            while True:
                x = input("start again?(type 'y')")
                if x == 'y':
                    break




def lead_to_room():
    """
    short 'demo' where Toya leads someone to the living room
    """
    with real_robot:
        TalkingMotion("i can show you the living room, if you like").resolve().perform()
        rospy.sleep(2)
        TalkingMotion("Just follow me").resolve().perform()
        rospy.sleep(1)

        # lead human to living room
        NavigateAction([pose_kitchen_to_couch]).resolve().perform()
        NavigateAction([pose_couch]).resolve().perform()

        rospy.sleep(1)
        TalkingMotion("Welcome to the living room").resolve().perform()


def hri():
    """
    testing HRI and introduction and navigating
    """

    with real_robot:

        global callback
        global response
        test_all = True

        rospy.Subscriber("nlp_out", String, data_cb)
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")

        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(0.9)

        # signal to start listening
        pub_nlp.publish("start listening")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>":
            if response[1] != "<None>":
                TalkingMotion("it is so noisy here, please confirm if i got your name right").resolve().perform()
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


if __name__ == '__main__':
    describe_human()
    # lead_to_room()
    # hri()
