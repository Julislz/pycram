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

rospy.sleep(8)
pub_nlp.publish(f"<GUEST>, Lucas, coffee")
callback = False

while not callback:
    rospy.sleep(1)

callback = False
rospy.sleep(1)
pub_nlp.publish(f"<CONFIRM>, True")

while not callback:
    rospy.sleep(1)

callback = False
rospy.sleep(1)
pub_nlp.publish(f"<CONFIRM>, True")