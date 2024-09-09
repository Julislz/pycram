from .default_process_modules import *


class DonbotMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("ur5_shoulder_link"))

        if pose_in_shoulder.position.x >= 0 and pose_in_shoulder.position.x >= abs(pose_in_shoulder.position.y):
            robot.set_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("left_arm", "front"))
        if pose_in_shoulder.position.y >= 0 and pose_in_shoulder.position.y >= abs(pose_in_shoulder.position.x):
            robot.set_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("left_arm", "arm_right"))
        if pose_in_shoulder.position.x <= 0 and abs(pose_in_shoulder.position.x) > abs(pose_in_shoulder.position.y):
            robot.set_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("left_arm", "back"))
        if pose_in_shoulder.position.y <= 0 and abs(pose_in_shoulder.position.y) > abs(pose_in_shoulder.position.x):
            robot.set_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("left_arm", "arm_left"))

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("ur5_shoulder_link"))

        new_pan = np.arctan2(pose_in_shoulder.position.y, pose_in_shoulder.position.x)

        robot.set_joint_position("ur5_shoulder_pan_joint", new_pan + robot.get_joint_position("ur5_shoulder_pan_joint"))


class DonbotManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "iai_donbot"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DonbotMoveHead(self._looking_lock)
