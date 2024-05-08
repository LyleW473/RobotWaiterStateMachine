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
        self.requests_counter = 0  # Pointer to which request we are expecting
        self.known_foods = set(["broccoli", "pizza", "sandwich"])

    def check_for_reset(self, userdata):
        # Out of sync
        if self.requests_counter < userdata.request_data["request_number"]:
            rospy.loginfo("Resetting 'WaitForRequest' state!")
            self.data = None
            self.requests_counter += 1

    def execute(self, userdata):
        # Note: Does not work if subscriber is not placed here:
        self.subscriber = rospy.Subscriber("/food_request", FoodRequest, self.receive_data, queue_size=1)

        # Check if we are now waiting for a new request (completed one iteration, so should reset).
        self.check_for_reset(userdata=userdata)

        if self.data is None:
            return "failed"
        else:
            # Check if the food is recognised
            food_name = self.data.food_name.lower()
            if food_name not in self.known_foods:
                return "failed"
            person_name = self.data.person_name.lower().capitalize()

            userdata.status_type = "succeeded"
            userdata.request_data["person_name"] = person_name
            userdata.request_data["food_name"] = food_name
            rospy.loginfo("Received data on topic /food_request!")
            rospy.loginfo(f" {self.data.person_name} | {self.data.food_name}")
            return "succeeded"


    def receive_data(self, data):
        self.data = data