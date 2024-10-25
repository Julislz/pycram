
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater

from pycram.enums import ObjectType

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher


def startup():
    world = BulletWorld()
    v = VizMarkerPublisher()

    #world = BulletWorld()
    v = VizMarkerPublisher()
    kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
    text_to_speech_publisher = TextToSpeechPublisher()
    image_switch_publisher = ImageSwitchPublisher()

    robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
    robot.set_color([0.5, 0.5, 0.9, 1])

    RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
    move = PoseNavigator()

    return world, v, text_to_speech_publisher, image_switch_publisher, move, robot
