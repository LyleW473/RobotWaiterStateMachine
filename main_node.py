#!/usr/bin/env python

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction
from states import *
from callbacks import *

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=["current_stage", "locations", "status_type"])

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
    sm.userdata.request_data = {"personName": None, "foodName": "None"}

    with sm:

        smach.StateMachine.add(
                               "STAGE_UPDATER",
                               StageUpdater(
                                            outcomes=["navigate", "wait_for_request", "food_detection", "completed"],
                                            input_keys=["current_stage", "locations", "set_location", "status_type"],
                                            output_keys=["current_stage", "set_location", "status_type"]
                                            ),
                               transitions={
                                            "navigate": "NAVIGATE",
                                            "wait_for_request": "WAIT_FOR_REQUEST",
                                            "food_detection": "YOLO_FOOD_DETECTION",
                                            "completed": "succeeded"
                                            },
                               remapping={
                                        "current_stage": "current_stage",
                                        "set_location": "set_location",
                                        "status_type":"status_type"
                                        }
                               )
        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState("move_base",
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

        smach.StateMachine.add("YOLO_FOOD_DETECTION",
                               YOLOFoodDetection(
                                                outcomes=["succeeded", "failed"],
                                                input_keys=["status_type", "request_data"],
                                                output_keys=["status_type"]
                                                ),
                               transitions={
                                            "succeeded":"STAGE_UPDATER",
                                            "failed": "YOLO_FOOD_DETECTION"
                                           },
                               remapping={
                                        "status_type": "status_type"
                                        }
                               )

        ## TO DO LIST: Fix the error with user data key stages not being available before state machine is run
        ## Implement waiting for request stage on topic
        ## ....

        rospy.loginfo(sm.userdata.current_stage)

    sis = IntrospectionServer("my_server", sm, "SM_ROOT")
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()