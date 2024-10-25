import rospy
from pycram.utilities.robocup_utils import StartSignalWaiter
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener
from pycram.designator import LocationDesignatorDescription
import random



# Create an instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()
#load everfyhing world giskard robokudo....
# Wait for the start signal
start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")
