
from pycram.bullet_world import BulletWorld, Object
from pycram.designator import ObjectDesignatorDescription
from pycram.local_transformer import LocalTransformer
from pycram.pose import Pose
import pycram.external_interfaces.giskard_new as giskardpy
import pycram.external_interfaces.navigate as navi
from pycram.ros.robot_state_updater import RobotStateUpdater

from pycram.utilities.robocup_utils import pakerino

world = BulletWorld()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
navPose = Pose([3,2,0])
move = navi.PoseNavigator()

giskardpy.init_giskard_interface()
giskardpy.clear()
config_for_placing = {'arm_lift_joint': 0.64, 'arm_flex_joint': -1.5, 'arm_roll_joint': 0, 'wrist_flex_joint': 0,
                      'wrist_roll_joint': 0}
pakerino()
lt = LocalTransformer()
mTb = lt.transform_pose(navPose, robot.get_link_tf_frame("base_link"))
mTb.pose.position.x -= 0.8

bTm = lt.transform_pose(mTb, "map")

print(navPose)
print(mTb)
print(bTm)
move.pub_now(bTm)
