import rospy
from std_msgs.msg import String

pub_nlp = rospy.Publisher('/nlp_out', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
needed = True

while needed:
    command = input("yes/no for Confirm, stop for stopping program, "
                    "nlp will publish Guest infos and then Confirms automatically,"
                    "everything else will publish name: ")
    if command == "stop":
        needed = False
    elif command == "yes":
        pub_nlp.publish(f"<CONFIRM>, True")
    elif command == "no":
        pub_nlp.publish(f"<CONFIRM>, False")
    elif command == "nlp":
        pub_nlp.publish(f"<GUEST>, Bob, coffee")
        for i in range(5):
            rospy.sleep(3)
            pub_nlp.publish(f"<CONFIRM>, True")
    else:
        pub_nlp.publish(f"<GUEST>, {command}, water")
