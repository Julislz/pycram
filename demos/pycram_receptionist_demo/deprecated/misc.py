from pycram.designators.motion_designator import TalkingMotion
from pycram.designators.object_designator import *
from pycram.helper import axis_angle_to_quaternion

# Publisher for NLP
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
pub_pose = rospy.Publisher('/human_pose', PoseStamped, queue_size=10)
understood_name = False
understood_drink = False
# TODO: set to False when NLP has implemented that feature
doorbell = True

# Pose variables
# Pose in front of the couch, HSR looks in direction of couch
pose_couch = Pose([2.7, 5, 0], [0, 0, 1, 0])
pose_door = Pose([1.4, 0.25, 0], [0, 0, 1, 0])

# Pose in the passage between kitchen and living room
robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.35, 3, 0], robot_orientation)


def talk_request(data: list):
    """
    function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: list ["name" "drink"]
    """
    global understood
    name, drink = data
    guest1.set_name(name)
    guest1.set_drink(drink)
    toyas_text = f"Hey {name}, your favorite drink is {drink}"

    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(1)
    TalkingMotion("Nice to meet you").resolve().perform()

    understood = True


def talk_request_nlp(data: String):
    """
    function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name, drink"
    """
    print(guest1.name)
    data_list = data.data.split(",")
    name, drink = data_list
    if guest1.name == "guest1":
        toyas_text = f"Hey {name}, your favorite drink is {drink}"

        # TalkingMotion(toyas_text).resolve().perform()
        rospy.sleep(2)

        guest1.set_name(name)
        guest1.set_drink(drink)
    elif not understood_name:
        toyas_text = f"is your name {name} ?"
        TalkingMotion(toyas_text).resolve().perform()
        pub_nlp.publish("start")
    elif understood_name and not understood_drink:
        toyas_text = f"is your favorite drink {drink} ?"
        TalkingMotion(toyas_text).resolve().perform()
        pub_nlp.publish("start")



def talk_error(data):
    """
    callback function if no name/drink was heard
    """
    error_msgs = "i could not hear you, please repeat"
    TalkingMotion(error_msgs).resolve().perform()
    rospy.sleep(2)
    pub_nlp.publish("start listening")


def introduce(name_a: Optional[str] = guest1.name, drink_a: Optional[str] = guest1.fav_drink, pose_a: Optional[PoseStamped] = None,
              name_b: Optional[str] = host.name, drink_b: Optional[str] = host.fav_drink, pose_b: Optional[PoseStamped] = None):
    """
    Text for robot to introduce two people to each other and alternate gaze
    :param name_a: name of the person that gets introduced to person b first
    :param drink_a: favorite drink of person a
    :param pose_a: position of person a where the hsr will look at
    :param name_b: name of the person that
    :param drink_b: favorite drink of person b
    :param pose_b: position of person b where the hsr will look at
    """
    # TODO: needs to be tested!
    if pose_b:
        pub_pose.publish(pose_b)
        rospy.sleep(3.5)
    TalkingMotion(f"Hey, {name_b}").resolve().perform()


    if pose_a:
        pub_pose.publish(pose_a)
        rospy.sleep(3.5)
    TalkingMotion(f" This is {name_a} and their favorite drink is {drink_a}").resolve().perform()
    rospy.sleep(2)
    TalkingMotion(f"Hey, {name_a}").resolve().perform()


    if pose_b:
        pub_pose.publish(pose_b)
        rospy.sleep(3.5)
    TalkingMotion(f" This is {name_b} and their favorite drink is {drink_b}").resolve().perform()

    rospy.sleep(3)


def doorbell_cb(data):
    """
    callback function for a subscriber to NLP script.
    is called when doorbell sound was detected
    """
    global doorbell
    doorbell = True


def name_cb(data):
    """
    callback function for a subscriber to NLP script.
    is called when name was correctly understood
    """
    global understood_name
    global understood_drink
    if data.data and not understood_name:
        TalkingMotion("perfect").resolve().perform()
        understood_name = True
    elif data.data and not understood_drink:
        TalkingMotion("alright, thank you and nice to meet you").resolve().perform()
        understood_drink = True
    else:
        TalkingMotion("i am sorry, please repeat yourself loud and clear").resolve().perform()
        rospy.sleep(1)
        # TODO: only hear for Name
        pub_nlp.publish("start listening")




