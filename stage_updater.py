import rospy
from smach import State

"""
State used to transition between different stages
"""
class StageUpdater(State):
    def __init__(self, outcomes, input_keys, output_keys):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

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