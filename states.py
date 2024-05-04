import actionlib
import rospy
import smach
from second_coursework.msg import FoodRequest
from yolo_service import YOLOService
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameRequest
from geometry_msgs.msg import Twist

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
        if userdata.current_stage == 0: # Used for localisation because of poor navigation without it
            userdata.rotate_to_localise = True
            return "rotate"
        if userdata.current_stage == 1:
            userdata.set_location = "TABLE"
            return "navigate"
        if userdata.current_stage == 2:
            return "wait_for_request"
        if userdata.current_stage == 3:
            userdata.set_location = "KITCHEN"
            return "navigate"
        if userdata.current_stage == 4:
            return "food_detection"
        # Note: ADD TTS for the person in the kitchen here:
        if userdata.current_stage == 5:
            userdata.set_location = "TABLE"
            return "navigate"
        if userdata.current_stage == 6:
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

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def execute(self, userdata):

        userdata.generating_detection = True

        # Send a request for detections
        request = YOLOLastFrameRequest()
        response = self.yolo_client(request)
        print(response.class_prediction, response.class_confidence)
        self.data = response
        userdata.generating_detection = False

        # Check if objective has been achieved
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


class RotateState(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def execute(self, userdata):

        # Rotate state used to localise robot (rotate for 2.5 seconds)
        if userdata.rotate_to_localise == True:
            rospy.loginfo("reached")
            old_time = rospy.get_time()
            while rospy.get_time() - old_time < 2.5:
                self.rotate_inplace(rotate_speed=1)

            userdata.rotate_to_localise = False
            userdata.status_type = "succeeded"
        else:
            # While detections are still being generated
            while userdata.generating_detection == True:
                rospy.loginfo(userdata.rotate_to_localise)
                # Rotate in place
                self.rotate_inplace(rotate_speed=5)
        rospy.loginfo("REACHED")
        return "succeeded"

    def rotate_inplace(self, rotate_speed=1):
        command = Twist()

        command.linear.x = 0
        command.linear.y = 0
        command.linear.z = 0

        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = rotate_speed

        self.velocity_publisher.publish(command)