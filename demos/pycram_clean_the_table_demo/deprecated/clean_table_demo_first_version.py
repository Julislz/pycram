from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy

from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.enums import ObjectType
from pycram.language import macros, par
import sys
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", ObjectType.ROBOT, "../../../resources/hsrb.urdf", pose=Pose([1, 2, 0]))
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
kitchen = Object("kitchen", "environment", "../../../resources/kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
# spawning_poses = {
#
# }


#giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()

with simulated_robot:
    dishwasher_desig = ObjectPart(names=["dishwasher"], part_of=kitchen_desig.resolve())
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform() #dishwasher

#
# waitfor door opening
#  navigate dishwasher
# #OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
# #open dishwasher
#  pull rack, (open rack)
#     NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
#     #navigate to table (might be round)
#     manipulation perceive pose
#     perceived_items[objectdesig] = perception call
#     spawn perceived_items
#     syn.world giskard
#     add.mesh to giskard
#     loop for items
#         design_bowl = perceived_items[i] if = Bowl
#         desig_mug =  ....
#
#     picku
#     bowl
#     navigate to dishwasher
#     place bowl in dishswer
#     navigate to table -> frag knowledge nach next good pose for objectX
#     pickup mug
#     navigate to dishwasher
#     place mug
#     close rack
#     open rack2
#     navigate to table -> knowledge ""
#     pickup spoon knife or fork z selfmade
#     navigate to dishwasher
#     place
#     navigate..
#     texttospeech: "pls give me the last object the plat that is on the table, ill open my gripper"
#     opengripper
#     text: "when u done please say it or move my gripper"
#     wait 5 sec
#     close griper
#     navigate dishwasher
#     place plate
#     close dishwasher
# done