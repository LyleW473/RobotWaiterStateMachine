import rospy
from smach import State
from second_coursework.msg import FoodRequest

"""
State to wait for request on the /food_request topic
"""
class WaitForRequest(State):
    def __init__(self, outcomes, input_keys, output_keys):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.data = None

    def execute(self, userdata):
        # Note: Does not work if subscriber is not placed here:
        self.subscriber = rospy.Subscriber("/food_request", FoodRequest, self.receive_data)

        if self.data is None:
            return "failed"
        else:
            userdata.status_type = "succeeded"
            userdata.request_data["person_name"] = self.data.person_name
            userdata.request_data["food_name"] = self.data.food_name
            rospy.loginfo("Received data on topic /food_request!")
            rospy.loginfo(f" {self.data.person_name} | {self.data.food_name}")
            return "succeeded"

    def receive_data(self, data):
        self.data = data