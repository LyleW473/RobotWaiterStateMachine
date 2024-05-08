import rospy
from smach import State
from second_coursework.msg import YOLOLastDetectPrediction
from geometry_msgs.msg import Twist

"""
State to detect food when inside of room D.
"""
class YOLOFoodDetection(State):
    def __init__(self, outcomes, input_keys, output_keys):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.last_detection = None
        self.detections_subscriber = rospy.Subscriber("/food_detections", YOLOLastDetectPrediction, self.detection_callback, queue_size=10)
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