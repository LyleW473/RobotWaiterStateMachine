import rospy
import smach
from std_msgs.msg import String

@smach.cb_interface(
    input_keys=["status_type", "speech_message"],
    output_keys=["status_type"],
    outcomes=["succeeded"]
)
def talk_cb(userdata, speech_publisher):
    # Publish message to talk
    message = String()
    message.data = userdata.speech_message
    speech_publisher.publish(message)

    # Sleep
    rospy.sleep(3)

    userdata.status_type = "succeeded"
    return "succeeded"