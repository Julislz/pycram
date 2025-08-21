import time
from std_msgs.msg import String
from demos.pycram_hri_study.utils.ResponseLoader import ResponseLoader
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import HumanDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None, None]
callback = False
timeout = 12


class NLP_Interface:
    """
    Class for enabling nlp on the robot,
    stores important nlp functions
    """

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.image_switch_publisher = ImageSwitchPublisher()

        # variables for response handling
        self.res_loader = ResponseLoader()
        self.res_loader.load_data()
        self.response = ["", ""]

        # variable to check if nlp result was published
        self.callback = False

    def data_cb(self, data):
        """
        function to receive data from nlp via /nlp_out topic
        """
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)

        # process result
        self.response = data.data.split(";")
        for ele in self.response:
            ele.strip()

        # log result and signal code to continue
        rospy.loginfo(self.response)
        self.callback = True

    def welcome_guest(self, guest: HumanDescription):
        """
        talking sequence to get name of a guest
        :param guest: variable to store new information about human
        """
        TalkingMotion("waiting for guests").perform()

        # look for human and position higher
        DetectAction(technique='human_receptionist').resolve().perform()
        TalkingMotion("please come closer").perform()
        rospy.sleep(2)
        TalkingMotion("thank you").perform()
        MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

        # look at guest and introduce
        HeadFollowMotion(state="start").perform()
        rospy.sleep(1)
        TalkingMotion("Hello, i am Toya").perform()
        rospy.sleep(1.1)
        TalkingMotion("What is your name?").perform()
        rospy.sleep(1.1)
        TalkingMotion("answer me when my display changes").perform()
        rospy.sleep(2.3)

        # signal to start listening
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.1)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        # wait for nlp answer
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            # failure handling if speech processing failed
            if int(time.time() - start_time) == timeout:
                start_time = time.time()
                rospy.logwarn("guest needs to repeat")

                # try listening again and signal guest to repeat themselves
                self.nlp_pub.publish("start listening")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        # reset variable for next use
        self.callback = False

        # check response -> was everything understood with right intent
        if self.response[0] == "<GUEST>":
            # success a name and intent was understood
            if self.response[1].strip() != "None":
                # understood both
                guest.set_name(self.response[1])
            else:
                # failed to understand name
                guest.set_name(self.name_repeat())

        else:
            # two chances to get name and drink
            guest.set_name(self.name_repeat())
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        TalkingMotion(f"Nice to meet you {guest.name}").perform()

        return guest

    def name_repeat(self):
        """
        HRI-function to ask for name again.
        """

        self.callback = False
        trys = 0

        while trys < 2:
            TalkingMotion("i am sorry, please repeat your name").perform()
            self.image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            rospy.sleep(2)
            TalkingMotion("use the sentence my name is").perform()
            rospy.sleep(1.2)

            self.nlp_pub.publish("start")
            rospy.sleep(2.5)
            self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

            # wait for response
            start_time = time.time()
            while not self.callback:

                # signal repeat to human
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                    self.nlp_pub.publish("start listening")
                    start_time = time.time()

            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[1].strip() != "None":
                return self.response[1]

            trys += 1

    def listen_return_answer(self):
        """
        function that returns whatever the person said
        """

        # signal to start listening
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        # wait for nlp answer
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        self.callback = False
        return response

    def listen_return_interest(self):
        """
        function that returns interests of conversational partner
        """
        TalkingMotion("answer me when my display changes").perform()
        trys = 0

        # wait for nlp answer
        start_time = time.time()

        # signal to start listening
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.1)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        while not self.callback and trys < 2:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                self.nlp_pub.publish("start listening")
                rospy.sleep(1.3)
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                trys += 1

        self.callback = False
        if self.response[0] == "<INTERESTS>" and self.response[1].strip() != "None":
            rospy.loginfo("interest understood")
            return eval(self.response[1])

        # failed to understand interest
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        return None

    def get_fav_drink(self, guest: HumanDescription):
        """
        sequence in which robot asks person for favorite drink and stores it
        :param guest: variable that stores favorite drink
        """
        TalkingMotion("What is your favorite drink?").perform()
        rospy.sleep(2)
        TalkingMotion("please answer me when my display changes").perform()
        rospy.sleep(2.2)

        # signal to start listening
        self.nlp_pub.publish("start listening")
        rospy.loginfo("nlp start")
        rospy.sleep(2.1)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        # wait for nlp answer
        start_time = time.time()
        trys = 0
        while not self.callback and trys < 2:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                self.nlp_pub.publish("start listening")
                start_time = time.time()
                rospy.sleep(1)
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                trys += 1

        self.callback = False

        # check response -> was everything understood with right intent
        if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
            guest.set_drink(self.response[2])
        else:
            # failure handling ask again
            guest.set_drink(self.drink_repeat())

        if guest.fav_drink:
            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            TalkingMotion(f"your favorite drink is {guest.fav_drink}").perform()

    def drink_repeat(self):
        """
        HRI-function to ask for drink again.
        """

        self.callback = False
        trys = 0

        while trys < 1:
            TalkingMotion("i am sorry, please repeat your drink loud and clear").perform()
            self.image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            rospy.sleep(3.5)
            TalkingMotion("please use the sentence my favorite drink is").perform()
            rospy.sleep(2.5)

            self.nlp_pub.publish("start")
            rospy.sleep(2)
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

            # wait for response
            start_time = time.time()
            while not self.callback and trys < 1:
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.nlp_pub.publish("start listening")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                    trys += 1

            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
                trys += 1
                return self.response[2]

        return False

    def store_and_answer_hobby(self, guest: HumanDescription):
        """
        function to answer to hobby individually
        """

        # get interests
        hobby_list = self.listen_return_interest()

        # store interests
        if hobby_list:
            # filter common nlp error - playing tennis is processed "playing" as hobby
            if hobby_list[0] == 'playing':
                hobby_list.remove('playing')
            guest.add_interests(hobby_list[0])

        # another hobby detected after removing "playing"
        if hobby_list:

            # process result for answer
            for indx in range(len(hobby_list)):
                hobby_list[indx] = hobby_list[indx].lower()

        # answer specifically or if hobby unknown a fallback answer
        toya_text = self.res_loader.predict_response(hobby_list)
        if guest.interests:
            TalkingMotion(f"you like {guest.interests[0]}").perform()
        TalkingMotion(toya_text).perform()
