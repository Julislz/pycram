import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.action_designator import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from docutils.nodes import math
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *



world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "couch-whole_kitchen2.urdf")
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
wished_sorted_obj_list = ["Metalbowl", "Cerealbox", "Milkpack", "Spoon", "Chips"]


with real_robot:
    print(robot.get_complete_joint_state())

    TalkingMotion("starting test").resolve().perform()
    rospy.sleep(2)
    desig = navigate_and_detect('table')
    sorted_obj = sort_obj(desig, robot)
    for obj in sorted_obj:

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        PickUpAction(obj, ["left"], ["front"]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveTorsoAction([0.05]).resolve().perform()
        MoveTorsoAction([0.1]).resolve().perform()

    TalkingMotion("test end").resolve().perform()

