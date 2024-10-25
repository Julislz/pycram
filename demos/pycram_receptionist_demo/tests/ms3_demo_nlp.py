import rospy
from std_msgs.msg import String

callback = False


def data_cb(data):
    global callback

    callback = True


pub_nlp = rospy.Publisher('/nlp_out', String, queue_size=10)
rospy.Subscriber("/startListener", String, data_cb)
rospy.init_node('talker', anonymous=True)

while not callback:
    rospy.sleep(1)

<<<<<<< HEAD
rospy.sleep(8)
pub_nlp.publish(f"<GUEST>, Lucas, coffee")
=======
rospy.sleep(9)
pub_nlp.publish(f"<GUEST>, Angel, coffee")
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
callback = False

while not callback:
    rospy.sleep(1)

callback = False
<<<<<<< HEAD
rospy.sleep(1)
=======
rospy.sleep(2)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
pub_nlp.publish(f"<CONFIRM>, True")

while not callback:
    rospy.sleep(1)

callback = False
<<<<<<< HEAD
rospy.sleep(1)
=======
rospy.sleep(2)
>>>>>>> a27749b26775a067b9d2b550387c2c08c00dadfa
pub_nlp.publish(f"<CONFIRM>, True")