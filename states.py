import actionlib
import rospy
import smach
from second_coursework.msg import FoodRequest, YOLOLastDetectPrediction
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
        elif userdata.current_stage == 1:
            userdata.set_location = "TABLE"
            return "navigate"
        elif userdata.current_stage == 2:
            return "wait_for_request"
        elif userdata.current_stage == 3:
            userdata.set_location = "KITCHEN"
            return "navigate"
        elif userdata.current_stage == 4:
            return "food_detection"
        elif userdata.current_stage == 5:
            msg = f'Please place the {userdata.request_data["food_name"]} on me!'
            userdata.speech_message = msg
            return "talk"
        elif userdata.current_stage == 6:
            userdata.set_location = "TABLE"
            return "navigate"
        elif userdata.current_stage == 7:
            msg = f'Hello {userdata.request_data["person_name"]}, here is your {userdata.request_data["food_name"]}.'
            userdata.speech_message = msg
            return "talk"
        elif userdata.current_stage == 8:
            return "completed"

    def reset_userdata(self, userdata):
        userdata.status_type = "unknown"
        userdata.set_location = None
        userdata.speech_message = ""


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
            userdata.request_data["person_name"] = self.data.person_name
            userdata.request_data["food_name"] = self.data.food_name
            rospy.loginfo("Received data on topic /food_request!")
            rospy.loginfo(f" {self.data.person_name} | {self.data.food_name}")
            return "succeeded"

    def receive_data(self, data):
        self.data = data

"""
State to detect food when inside of room D.
"""
class YOLOFoodDetection(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.last_detection = None
        self.detections_subscriber = rospy.Subscriber("/food_detections", YOLOLastDetectPrediction, self.detection_callback)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def detection_callback(self, message):
        self.last_detection = message

    def execute(self, userdata):

        # Check if objective has been achieved
        if self.last_detection is None:
            rospy.loginfo(self.last_detection)
            rospy.loginfo("No detection found")
            return "failed"
        else:
            print(self.last_detection)
            print(self.last_detection.class_prediction, userdata.request_data["food_name"].lower())
            rospy.loginfo("Found detection")
            # Check whether the food found is the one requested by the user
            if self.last_detection.class_prediction == userdata.request_data["food_name"].lower():
                rospy.loginfo(self.last_detection.class_prediction == userdata.request_data["food_name"].lower())
                rospy.loginfo("Found correct food!")
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
            rospy.loginfo(userdata.rotate_to_localise)
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

class TalkState(smach.State):
    def __init__(self, outcomes, input_keys, output_keys):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.speech_publisher = rospy.Publisher("/tts/phrase", String, queue_size=10)

    def execute(self, userdata):
        message = String()
        message.data = userdata.speech_message
        self.speech_publisher.publish(message)
        rospy.sleep(5)

        userdata.status_type = "succeeded"
        return "succeeded"