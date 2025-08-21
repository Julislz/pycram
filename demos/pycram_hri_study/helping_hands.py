from typing import Optional
import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from demos.pycram_receptionist_demo.utils.helper import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import rospy
from pycram.utilities.robocup_utils import TextToImagePublisher, ImageSwitchPublisher

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()
text_to_img_publisher = TextToImagePublisher()
img = ImageSwitchPublisher()
navigation = PoseNavigator()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = create_action_client('robokudo/query', QueryAction)
rkclient.wait_for_server()


class Human:
    """
    Class that represents humans.
    To check if we know where a humans pose is
    """

    def __init__(self):
        self.human_pose = False

        # Subscriber to the human pose topic
        self.human_pose_sub = rospy.Subscriber("/human_pose", PointStamped, self.human_pose_cb)

    def human_pose_cb(self):
        """
        Callback function for human_pose Subscriber.
        Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
        """
        self.human_pose = True


human = Human()
second_timer_pose = None
timeout1 = 14


def demo(step: int, clear_path: Optional[bool] = True):

    with (real_robot):

        if step <= 1:
            TalkingMotion("I am excited for the next interaction").perform()
            MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
            img.pub_now(ImageEnum.HI.value)

            # move robot in starting position
            MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()

            # wait for human and hand to be pushed down
            look_human(human)

        if step <= 2:
            TalkingMotion("when we arrive, push down my gripper.").perform()
            rospy.sleep(2.5)
            TalkingMotion("please walk slowly i will follow you now").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)

            try:
                # follow human until gripper is pushed down
                plan = Code(lambda: giskardpy.cml(drive_back=False, clear_path=clear_path)) >> Monitor(monitor_func)
                plan.perform()

            except SensorMonitoringCondition:

                # location of bag is reached
                MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                TalkingMotion("We have arrived.").perform()
                MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()
                TalkingMotion("Please hand the bag in my gripper").perform()

                # show instructions on display
                text_to_img_publisher.pub_now("when the bag is handed in push down my gripper")
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
                rospy.sleep(3)
                img.pub_now(ImageEnum.GENERATED_TEXT.value)
                TalkingMotion("please put the bag in my gripper and push down my gripper").perform()

                try:
                    # wait until bag is placed in gripper - gripper is pushed down
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()

                except SensorMonitoringCondition:
                    MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                    TalkingMotion("Closing my Gripper.").perform()
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                    rospy.sleep(1)
                    TalkingMotion("i will bring it to the kitchen for you").perform()

                    if step <= 3:
                        # drive in the kitchen
                        drive_back_move_base()

                        # place bag on the floor
                        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
                        MoveJointsMotion(["arm_flex_joint"], [-0.6]).perform()
                        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
                        img.pub_now(ImageEnum.HI.value)

            except giskardpy.ExecutionException:

                # exception handling, when robot lost human
                TalkingMotion("Wait").perform()
                rospy.sleep(1)
                TalkingMotion("i lost sight of you").perform()
                rospy.sleep(1)
                TalkingMotion("Please come back").perform()
                rospy.sleep(1)
                MoveJointsMotion(["head_tilt_joint"], [0.2]).perform()
                MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
                look_human(human=human)
                demo(2, clear_path=False)


def look_human(human: Human):
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """

    try:
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        TalkingMotion("Push down my Hand, when i should follow you").perform()

        # wait for gripper to be pushed down
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()

    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.SEARCH.value)
        TalkingMotion("Please step in front of me").perform()
        human.human_pose = False

        # look for human
        goal_msg = QueryGoal()
        x = rkclient.send_goal(goal_msg)

        # failure handling if no human is detected
        rospy.loginfo("Waiting for human to be detected")
        start_time = time.time()
        timeout = 5
        timeout2 = 15

        while not human.human_pose:
            if time.time() - start_time >= timeout:
                rkclient.send_goal(goal_msg)
            if time.time() - start_time >= timeout2:
                TalkingMotion("please step in front of me").perform()
                start_time = time.time()

        TalkingMotion("thank you").perform()
        img.pub_now(ImageEnum.HI.value)
        return


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 18.30:
        rospy.logwarn("sensor exception, gripper pushed")
        return SensorMonitoringCondition

    return False


def drive_back_move_base():
    """
    navigate with move base to the kitchen
    """
    nav_pose_1 = Pose([-0.2, -0.9, 0], orientation=[0, 0, 0, 1])
    nav_pose_2 = Pose([1.8, -1, 0], orientation=[0, 0, 0, 1])
    nav_pose_3 = Pose([3.5, -2, 0], orientation=[0, 0, 0, 1])

    NavigateAction([nav_pose_1]).resolve().perform()
    NavigateAction([nav_pose_2]).resolve().perform()
    TalkingMotion("almost there").perform()
    NavigateAction([nav_pose_3]).resolve().perform()
    TalkingMotion("task completed").perform()


demo(0)
