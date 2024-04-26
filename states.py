import rospy
import smach
from second_coursework.msg import FoodRequest

"""
State used to transition between different stages
"""
class StageUpdater(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    def execute(self, userdata):
        rospy.loginfo("StageUpdater")
        rospy.loginfo(f"Current stage: {userdata.current_stage} | Status: {userdata.status_type}")

        # Last stage was successful or is the initial state, otherwise keep repeating
        if userdata.status_type == "succeeded" or userdata.status_type == "initial":
            userdata.current_stage += 1
            rospy.loginfo(f"Advancing to stage: {userdata.current_stage}")

        # Reset status type
        userdata.status_type = "unknown"

        # Change events depending on whats needed
        if userdata.current_stage == 0:
            return "navigate"
        if userdata.current_stage == 1:
            return "wait_for_request"
        if userdata.current_stage == 2:
            return "navigate"
        if userdata.current_stage == 3:
            return "completed"


"""
State to wait for request on the /food_request topic
"""
class WaitForRequest(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.data = None

    def execute(self, userdata):
        # Note: Does not work if subscriber is not placed here:
        self.subscriber = rospy.Subscriber("/food_request", FoodRequest, self.receive_data)

        if self.data is None:
            return "failed"
        else:
            userdata.status_type = "succeeded"
            rospy.loginfo("Received data on topic /food_request!")
            rospy.loginfo(f" {self.data.personName} | {self.data.foodName}")
            return "succeeded"

    def receive_data(self, data):
        self.data = data