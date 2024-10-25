import rospy

import pycram.external_interfaces.giskard_new as giskardpy
from pycram.bullet_world import BulletWorld, Object

from pycram.local_transformer import LocalTransformer
from pycram.ros.robot_state_updater import RobotStateUpdater

from pycram.utilities.robocup_utils import pakerino, HSRBMoveGripperReal, GraspListener

giskardpy.init_giskard_interface()
gripper = HSRBMoveGripperReal()
config_for_placing = {
    'arm_flex_joint': -1.2,
    'arm_lift_joint': 1.15,
    'arm_roll_joint': 0,
    # 'head_pan_joint': 0.1016423434842566,
    # 'head_tilt_joint': 0.0255536193297764,
    'wrist_flex_joint': -1.6,
    'wrist_roll_joint': 0,
}
grasp_listener = GraspListener()
world = BulletWorld()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")

#
# pakerino()
# pakerino(config=config_for_placing)
rospy.sleep(2)
lt = LocalTransformer()
gripper.pub_now("open")
rospy.sleep(2)
print(grasp_listener.check_grasp())
rospy.sleep(2)
gripper.pub_now("close")
rospy.sleep(2)
print(grasp_listener.check_grasp())
gripper.pub_now("open")
rospy.sleep(2)
print(grasp_listener.check_grasp())
rospy.sleep(2)
gripper.pub_now("close")
rospy.sleep(2)
print(grasp_listener.check_grasp())
gripper.pub_now("open")
rospy.sleep(2)
print(grasp_listener.check_grasp())
rospy.sleep(2)
gripper.pub_now("close")
rospy.sleep(2)
print(grasp_listener.check_grasp())

