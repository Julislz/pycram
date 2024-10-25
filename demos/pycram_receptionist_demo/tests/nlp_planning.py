<<<<<<< HEAD
=======
import rospy

>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
<<<<<<< HEAD
from std_msgs.msg import String, Bool
=======
from std_msgs.msg import String, Bool, UInt16
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_8.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])



# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
<<<<<<< HEAD
response = ""
callback = False
doorbell = True

# Declare variables for humans
host = HumanDescription("Leonie", fav_drink="water")
guest1 = HumanDescription("guest1")
=======
#pub_color = rospy.Publisher('/hsrb/command_status_led', UInt16, queue_size=5, latch=True)
response = ""
callback = False
doorbell = False

# Declare variables for humans
host = HumanDescription("Alina", fav_drink="water")
guest = HumanDescription("guest1")
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
guest2 = HumanDescription("guest2")
seat_number = 2


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
<<<<<<< HEAD
    callback = True


def misc_fct():
    global response
    global callback

    rospy.Subscriber("/nlp_out", String, data_cb)

    with real_robot:

        #DetectAction(technique='human').resolve().perform()

        # look at guest and introduce
        #HeadFollowAction('start').resolve().perform()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(1)

        # signal to start listening
        pub_nlp.publish("start listening")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>":
            # success a name and intent was understood
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
            print(guest1.fav_drink)

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
=======
    print(response)
    callback = True


def doorbell_cb(data):
    global doorbell
    doorbell = True


def welcome_guest(num, guest: HumanDescription):
    """
    talking sequence to get name and favorite drink of guest
    and attributes if it is the first guest
    :param num: number of guest
    :param guest: variable to store information in
    """
    global callback
    callback = False
    # look for human
    DetectAction(technique='human').resolve().perform()

    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil").resolve().perform()
    rospy.sleep(3)
    TalkingMotion("please answer me after the sound").resolve().perform()
    rospy.sleep(2)
    TalkingMotion("What is your name and favorite drink?").resolve().perform()
    rospy.sleep(2)

    # signal to start listening
    pub_nlp.publish("start listening")
    rospy.sleep(2)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)
    sound_publisher.publish_sound_request()

    # wait for nlp answer
    while not callback:
        rospy.sleep(1)
    callback = False

    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1] != "<None>":
            TalkingMotion("please confirm if i got your name right").resolve().perform()
            guest.set_drink(response[2])
            rospy.sleep(1)
            guest.set_name(name_confirm(response[1]))

        else:
            # save heard drink
            if response[2]:
                guest.set_drink(response[2])

            # ask for name again once
            guest.set_name(name_repeat())

        # confirm favorite drink
        guest.set_drink(drink_confirm(guest.fav_drink))

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            TalkingMotion("please repeat your name and drink loud and clear").resolve().perform()
            pub_nlp.publish("start")
            rospy.sleep(1.5)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)
            sound_publisher.publish_sound_request()

            start_time = time.time()
            while not callback:
                rospy.sleep(1)
                if time.time() - start_time == timeout:
                    print("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            callback = False

            if response[0] == "<GUEST>":
                guest.set_name(response[1])
                guest.set_drink(response[2])
                break
            else:
                i += 1

def misc_fct():
    global response
    global callback

    rospy.Subscriber("/nlp_out", String, data_cb)

    with real_robot:
        welcome_guest(1, guest)
        rospy.sleep(3)
        TalkingMotion("next one").resolve().perform()
        rospy.sleep(3)
        welcome_guest(2, guest2)
        rospy.sleep(3)
        introduce(guest, guest2)


>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa


misc_fct()
