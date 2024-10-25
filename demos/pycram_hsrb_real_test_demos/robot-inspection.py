import giskardpy
import rospy

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter
import time

import actionlib
import rospy
from geometry_msgs.msg import PointStamped, Twist
from robokudo_msgs.msg import QueryAction, QueryGoal

import pycram.external_interfaces.giskard_new as giskardpy
from pycram.designators.action_designator import fts
from pycram.enums import ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.process_module import real_robot
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, HSRBMoveGripperReal, pakerino
from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest


def monitor_func():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


talk = TextToSpeechPublisher()
img = ImageSwitchPublisher()
gripper = HSRBMoveGripperReal()
move = PoseNavigator()
# robot = Object("hsrb", "robot", ".
giskardpy.init_giskard_interface()
giskardpy.clear()
# Create an instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()


def set_parameters(new_parameters):
    rospy.wait_for_service('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters')
    try:
        reconfigure_service = rospy.ServiceProxy('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters',
                                                 Reconfigure)
        config = Config()

        # Set the new parameters
        if 'forbid_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='forbid_radius', value=new_parameters['forbid_radius']))
        if 'obstacle_occupancy' in new_parameters:
            config.ints.append(IntParameter(name='obstacle_occupancy', value=new_parameters['obstacle_occupancy']))
        if 'obstacle_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='obstacle_radius', value=new_parameters['obstacle_radius']))

        # Empty parameters that are not being set
        config.bools.append(BoolParameter(name='', value=False))
        config.strs.append(StrParameter(name='', value=''))
        config.groups.append(GroupState(name='', state=False, id=0, parent=0))

        req = ReconfigureRequest(config=config)
        reconfigure_service(req)
        rospy.loginfo("Parameters updated successfully")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def move_vel(speed, distance, isForward, angle=0):
    # Starts a new node
    velocity_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    vel_msg = Twist()

    # Checking if the movement is forward or backwards
    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = 0
    if angle > 0:
        vel_msg.angular.z = angle
    else:
        vel_msg.angular.z = 0
    # Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0


    # Setting the current time for distance calculation
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # Loop to move the turtle a specified distance
    while not rospy.is_shutdown() and current_distance < distance:
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Take actual time to velocity calculation
        t1 = rospy.Time.now().to_sec()
        # Calculate distance
        current_distance = speed * (t1 - t0)

    # After the loop, stop the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)
# new_parameters = {
#     'forbid_radius': 0.15,
#     'obstacle_occupancy': 2,
#     'obstacle_radius': 0.15
# }
#
# set_parameters(new_parameters)
#


# move_vel(2, 2, False, 0.2)
# rospy.sleep(2)
# move_vel(2, 2, False, -0.4)
#move_vel(1, 1, False, -0.02)
#
# fake_pose = Pose([3.39,0.34,0])
# move.pub_fake_pose(fake_pose)

try:

    img.pub_now(ImageEnum.HI.value)  # hi im toya
    talk.pub_now("Push down my Hand, when you are Ready.")
    img.pub_now(ImageEnum.PUSHBUTTONS.value)
    plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
    plan.perform()
except SensorMonitoringCondition:
    talk.pub_now("Starting the Robot Inspection.")
    # load everfyhing world giskard robokudo....
    # Wait for the start signal

    img.pub_now(ImageEnum.INSPECT.value)
    start_signal_waiter.wait_for_startsignal()

    # Once the start signal is received, continue with the rest of the script
    rospy.loginfo("Start signal received, now proceeding with tasks.")
    move_vel(speed=2, distance=4, isForward=True)
    rospy.sleep(2)
    new_parameters = {
        'forbid_radius': 0.2,
        'obstacle_occupancy': 5,
        'obstacle_radius': 0.2
    }

    set_parameters(new_parameters)

    rospy.sleep(1)
    move_vel(0.2, 2, False, 0.03)
    fake_pose_2 = Pose([4, 0.3, 0])
    move.pub_fake_pose(fake_pose_2)
    talk.pub_now("Driving to the Inspection Point.")



    #nav_pose0 = Pose([1, 0.18, 0])
    #move.pub_now(nav_pose0)
    #move.pub_now(nav_pose0)
    # hi im toya
    nav_pose = Pose([6.2, 0, 0])
    move.pub_now(nav_pose)
    nav_pose2 = Pose([9.3, 1.1, 0], [0, 0, 0.7, 0.7])
    move.pub_now(nav_pose2)
    talk.pub_now("Inspection point arrived!.")

    try:

        img.pub_now(ImageEnum.HI.value)  # hi im toya
        talk.pub_now("Push down my Hand, when you are ready.")
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        talk.pub_now("Driving away be carefully.")

        move.pub_now(nav_pose2)
        nav_pose3 = Pose([5.6, 3.7, 0], [0, 0, 1, 0])
        move.pub_now(nav_pose3)
        nav_pose4 = Pose([4.6, 6.2, 0], [0, 0, 0.7, 0.7])
        move.pub_now(nav_pose4)



#move_vel(speed=1, distance=3, isForward=True)
