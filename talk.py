import rospy
from smach import State
from std_msgs.msg import String

class TalkState(State):
    def __init__(self, outcomes, input_keys, output_keys):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.speech_publisher = rospy.Publisher("/tts/phrase", String, queue_size=10)

    def execute(self, userdata):
        message = String()
        message.data = userdata.speech_message
        self.speech_publisher.publish(message)
        rospy.sleep(3)

        userdata.status_type = "succeeded"
        return "succeeded"