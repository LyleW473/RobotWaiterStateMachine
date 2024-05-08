#!/usr/bin/env python3

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String

# Import states
from rotate import RotateState
from stage_updater import StageUpdater
from food_detection import YOLOFoodDetection
from wait_for_request import WaitForRequest

from navigation_callbacks import navigation_goal_cb, navigation_result_cb
from speech_callback import talk_cb

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(
                            outcomes=['succeeded', 'aborted'],
                            # input_keys=["current_stage", "locations", "status_type", "request_data"]
                            )

    sm.userdata.current_stage = -1
    sm.userdata.locations = {
        "TABLE": {
            "position": {"x": 6.14, "y": 3.198230504989624},
            "orientation": {"z": 0.71, "w": 0.71}
        },
        "KITCHEN": {
            "position": {"x": 1.956, "y": 2.90412},
            "orientation": {"z": 0.71, "w": 0.71}
        }
    }
    sm.userdata.status_type = "initial"  # Default (Look at status types). Tracks if a stage was completed successfully or not
    sm.userdata.request_data = {"person_name": None, "food_name": None, "request_number": 0}
    sm.userdata.rotate_to_localise = False
    sm.userdata.speech_message = ""

    with sm:

        smach.StateMachine.add(
                               "STAGE_UPDATER",
                               StageUpdater(
                                            outcomes=["rotate", "navigate", "wait_for_request", "food_detection","talk", "completed"],
                                            input_keys=["current_stage", "locations", "set_location", "status_type", "rotate_to_localise", "speech_message", "request_data"],
                                            output_keys=["current_stage", "set_location", "status_type", "rotate_to_localise", "speech_message"]
                                            ),
                               transitions={
                                            "rotate": "ROTATE",
                                            "navigate": "NAVIGATE",
                                            "wait_for_request": "WAIT_FOR_REQUEST",
                                            "food_detection": "YOLO_FOOD_DETECTION",
                                            "talk": "TALK",
                                            "completed": "succeeded"
                                            },
                               remapping={
                                        "current_stage": "current_stage",
                                        "set_location": "set_location",
                                        "status_type": "status_type",
                                        "rotate_to_localise": "rotate_to_localise",
                                        "speech_message": "speech_message"
                                        }
                               )
        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState(
                                                "move_base",
                                                MoveBaseAction,
                                                goal_cb=navigation_goal_cb,
                                                result_cb=navigation_result_cb,
                                                input_keys = ["current_stage", "locations", "set_location"],
                                                output_keys = ["status_type"]
                                                ),
                               transitions = {
                                            "succeeded": "STAGE_UPDATER",
                                            "aborted": "STAGE_UPDATER",
                                            "preempted": "STAGE_UPDATER"
                                            },
                               remapping={
                                        "status_type": "status_type",
                                        }
                               )
        smach.StateMachine.add("ROTATE",
                              RotateState(
                                  outcomes=["succeeded"],
                                  input_keys=["rotate_to_localise", "status_type"],
                                  output_keys=["rotate_to_localise", "status_type"],
                              ),
                              transitions={
                                            "succeeded": "STAGE_UPDATER",
                                            },
                              remapping={
                                  "rotate_to_localise": "rotate_to_localise",
                                  "status_type": "status_type"}
                              )
        smach.StateMachine.add("WAIT_FOR_REQUEST",
                               WaitForRequest(
                                            outcomes=["succeeded", "failed"],
                                            input_keys=["status_type", "request_data"],
                                            output_keys=["status_type", "request_data"]
                                            ),
                               transitions={
                                            "succeeded":"STAGE_UPDATER",
                                            "failed": "WAIT_FOR_REQUEST"
                                           },
                               remapping={
                                        "status_type": "status_type",
                                        "request_data": "request_data"
                                        }
                               )

        text_speech_publisher = rospy.Publisher("/tts/phrase", String, queue_size=10)
        smach.StateMachine.add("TALK",
                               smach.CBState(
                                            talk_cb,
                                            cb_kwargs={"speech_publisher": text_speech_publisher}
                                            ),
                               transitions={
                                            "succeeded":"STAGE_UPDATER",
                                           },
                               remapping={
                                        "status_type": "status_type"
                                        }
                               )

        def food_detection_child_term_cb(outcome_map):
            # Stop everything
            if outcome_map["DETECT_FOOD"] == "succeeded":
                return True
            # Keep rotating + looking for detections
            return False

        def food_detection_outcome_cb(outcome_map):
            if outcome_map["DETECT_FOOD"] == "succeeded":
                return "succeeded"  # Finish concurrence
            else:
                return "failed"

        # Concurrence for YOLO Detection
        cc_food_detection = smach.Concurrence(
                                            outcomes=["succeeded", "failed"],
                                            default_outcome="failed",
                                            input_keys=["request_data", "status_type", "rotate_to_localise"],
                                            output_keys=["status_type"],
                                            outcome_map={
                                                        "succeeded": {
                                                                    "DETECT_FOOD": "succeeded"
                                                                    },
                                                        "failed": {
                                                                    "DETECT_FOOD": "failed"
                                                                    }
                                                        },
                                            child_termination_cb=food_detection_child_term_cb,
                                            outcome_cb=food_detection_outcome_cb
                                            )
        with cc_food_detection:

            cc_food_detection.add("ROTATE_STATE_FOOD",
                                   RotateState(
                                            outcomes=["succeeded"],
                                            input_keys=["rotate_to_localise"],
                                            output_keys=[],
                                            )
                                   )

            cc_food_detection.add("DETECT_FOOD",
                                   YOLOFoodDetection(
                                                    outcomes=["succeeded", "failed"],
                                                    input_keys=["status_type", "request_data"],
                                                    output_keys=["status_type"]
                                                    ),
                                   remapping={
                                            "status_type": "status_type",
                                            }
                                   )
        smach.StateMachine.add("YOLO_FOOD_DETECTION",
                               cc_food_detection,
                               transitions={
                                            "succeeded":"STAGE_UPDATER",
                                            "failed": "YOLO_FOOD_DETECTION"
                                           },
                               remapping={
                                        "status_type": "status_type",
                                        }
                             )

        ## TO DO LIST: Fix the error with user data key stages not being available before state machine is run
        ## - Move callback functions into callback.py
        ## - Add comments to states.py and remove the highlighted comment in the #TODO

        rospy.loginfo(sm.userdata.current_stage)

    sis = IntrospectionServer("my_server", sm, "SM_ROOT")
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()