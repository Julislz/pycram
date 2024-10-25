import rospy
<<<<<<< HEAD

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
=======
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.external_interfaces.robokudo import faces_queryHuman
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
from pycram.process_module import semi_real_robot, real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from demos.pycram_receptionist_demo.utils.new_misc import *
<<<<<<< HEAD
=======
from pycram.helper import axis_angle_to_quaternion


>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# careful that u spawn the correct kitchen
<<<<<<< HEAD
kitchen = Object("kitchen", "environment", "suturo_lab_version_2.urdf")
giskardpy.init_giskard_interface()
guest1 = HumanDescription("guest1")

#for obj in world.current_bullet_world.objects:
#   print(obj)

#kitchen.set_joint_state("iai_kitchen:arena:door_origin_revolute_joint", 1)

def p():
    with real_robot:
        seat = True
        attributes = False

        if attributes:
            # to signal the start of demo
            # TalkingMotion("Hello, i am ready for the test").resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

            TalkingMotion("Welcome, please step in").resolve().perform()
            MoveTorsoAction([0.1]).resolve().perform()

            TalkingMotion("detecting human now").resolve().perform()
            rospy.sleep(2)
            desig = DetectAction(technique='human', state='start').resolve().perform()
            HeadFollowAction('start').resolve().perform()
            pub_nlp.publish("start listening")
            # desig = DetectAction(technique='attributes').resolve().perform()


            desig2 = DetectAction(technique='attributes').resolve().perform()
            print("msgs from PPP: " + str(desig2))

            desig = DetectAction(technique='human', state='start').resolve().perform()

            HeadFollowAction('start').resolve().perform()
            rospy.sleep(4)
            desig = DetectAction(technique='human', state='stop').resolve().perform()

            rospy.sleep(4)
            TalkingMotion("detecting human now").resolve().perform()
            desig = DetectAction(technique='human', state='start').resolve().perform()
            #print(desig[1].pose)
            #guest1.set_pose(desig[1])

            # PointingMotion(guest1.pose.position.x, guest1.pose.position.y, guest1.pose.position.z).resolve().perform()
            #print("msgs from PPP: " + str(desig))
            #if desig != "False":
            # guest1.set_attributes(desig)

            # describe(guest1)
               # rospy.sleep(2)
            rospy.sleep(3)
            TalkingMotion("attributes over").resolve().perform()
            desig = DetectAction(technique='human', state='stop').resolve().perform()


            #DetectAction(technique='human', state='stop').resolve().perform()

            #pub_pose.publish(guest1.pose)
            # desig = DetectAction(technique='human', state='stop').resolve().perform()

        if seat:
            # new Query for free seat
            #TalkingMotion("detecting free seat on whole couch now").resolve().perform()
            seat = DetectAction(technique='location', state="sofa").resolve().perform()
            rospy.loginfo(seat[1])
            #rospy.sleep(2)
=======
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
giskardpy.init_giskard_interface()
host = HumanDescription("Lukas", fav_drink="water")
guest1 = HumanDescription("Jule", fav_drink="tea")

#for obj in world.current_bullet_world.objects:
#    print(obj)

def pakerino(torso_z=0.15):
    config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")

# kitchen.set_joint_state("iai_kitchen:arena:door_origin_revolute_joint", 1)

def p():
    with real_robot:
        seat = False
        attributes = True

        if attributes:

            # to signal the start of demo
            # TalkingMotion("Hello, i am ready for the pp.py").resolve().perform()
            #ParkArmsAction([Arms.LEFT]).resolve().perform()

            TalkingMotion("Test").resolve().perform()
            #pakerino(0.15)

            TalkingMotion("detecting human now").resolve().perform()
            desig = DetectAction(technique='human', state='start').resolve().perform()
            giskardpy.move_head_to_human()
            #pub_nlp.publish("start listening")
            # desig = DetectAction(technique='attributes').resolve().perform()
            #rospy.sleep(7)

            try:
                # remember face
                #keys = DetectAction(technique='human', state='face').resolve().perform()[1]
                #print(keys)
                #new_id = keys["keys"][0]
                #guest1.set_id(new_id)
                #print(new_id)

                #TalkingMotion("attributes now now").resolve().perform()
                rospy.sleep(2)


                # get clothes and gender
                attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
                guest1.set_attributes(attr_list)
                rospy.loginfo(attr_list)

            except KeyError:
                print("error")

            #HeadFollowAction('start').resolve().perform()
            print("keee")
            #print(keys[new_id])
            #pub_pose.publish(keys[new_id])

            rospy.sleep(2)
            TalkingMotion("end").resolve().perform()


        if seat:
            # new Query for free seat
            # TalkingMotion("detecting free seat on whole couch now").resolve().perform()
            seat = DetectAction(technique='location', state="sofa").resolve().perform()
            rospy.loginfo(seat[1])
            # rospy.sleep(2)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
            for place in seat[1]:
                print(place)
                print(place[0])
                if place[0] == 'False':
                    PointingMotion(float(place[1]), float(place[2]), float(place[3])).resolve().perform()
                    print("free")

<<<<<<< HEAD

            #print("########################")
            #print(seat[1][1][1])
            #if seat[1][0][0] == 'False':
                #print(seat[1][0][1])
                #PointingMotion(float(seat[1][1][1]), float(seat[1][1][2]), float(seat[1][1][3])).resolve().perform()




if __name__ == '__main__':
     p()
=======
            # print("########################")
            # print(seat[1][1][1])
            # if seat[1][0][0] == 'False':
            # print(seat[1][0][1])
            # PointingMotion(float(seat[1][1][1]), float(seat[1][1][2]), float(seat[1][1][3])).resolve().perform()


def ms3_perception():
    with real_robot:
        TalkingMotion("start").resolve().perform()


        HeadFollowAction('start').resolve().perform()
        host_pose = DetectAction(technique='human').resolve().perform()
        print(host_pose)

        rospy.sleep(10)

        TalkingMotion("end").resolve().perform()


if __name__ == '__main__':
    p()
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
