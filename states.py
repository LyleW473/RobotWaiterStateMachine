import rospy
import smach
from second_coursework.msg import FoodRequest
from yolo_service import YOLOService
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameRequest

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

        # Reset user data
        self.reset_userdata(userdata)

        # Change events depending on whats needed
        if userdata.current_stage == 0:
            userdata.set_location = "TABLE"
            return "navigate"
        if userdata.current_stage == 1:
            return "wait_for_request"
        if userdata.current_stage == 2:
            userdata.set_location = "KITCHEN"
            return "navigate"
        if userdata.current_stage == 3:
            return "food_detection"
        if userdata.current_stage == 4:
            return "completed"

    def reset_userdata(self, userdata):
        userdata.status_type = "unknown"
        userdata.set_location = None


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
            userdata.request_data["personName"] = self.data.personName
            userdata.request_data["foodName"] = self.data.foodName
            rospy.loginfo("Received data on topic /food_request!")
            rospy.loginfo(f" {self.data.personName} | {self.data.foodName}")
            return "succeeded"

    def receive_data(self, data):
        self.data = data

"""
State to detect food when inside of room D.
"""
class YOLOFoodDetection(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.data = None

        self.yolo_service = YOLOService()
        self.yolo_client = rospy.ServiceProxy("/detect_food", YOLOLastFrame)

    def execute(self, userdata):

        # Send a request for detections
        request = YOLOLastFrameRequest()
        response = self.yolo_client(request)
        print(response.class_prediction, response.class_confidence)
        self.data = response

        if self.data is None:
            return "failed"

        else:
            # Check whether the food found is the one requested by the user
            if response.class_prediction == userdata.request_data["foodName"].lower():
                print("Found correct food!")
                userdata.status_type = "succeeded"
                return "succeeded"
            else:
                return "failed"

    def receive_data(self, data):
        self.data = data
