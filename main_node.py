#!/usr/bin/env python

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from states import *

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=["current_stage", "stages"])

    with sm:

        sm.userdata.current_stage = -1
        sm.userdata.stages = [
                                {
                                 "position":{"x": 6.14, "y": 3.198230504989624},
                                 "orientation":{"z": -0.71, "w": 0.71}
                                },

                                {
                                    "position": {"x": 6.14, "y": 8.42},
                                    "orientation": {"z": -0.71, "w": 0.71}
                                },

                                {
                                    "position": {"x": 1.956, "y": 2.90412},
                                    "orientation": {"z": -0.71, "w": 0.71}
                                }
                                ]
        sm.userdata.status_type = "initial" # Default (Look at status types). Tracks if a stage was completed successfully or not


        def navigation_goal_cb(userdata, goal):

            rospy.loginfo(userdata.keys())
            current_stage = userdata.current_stage

            my_goal = MoveBaseGoal()

            # Header
            my_goal.target_pose.header.frame_id = "map"
            my_goal.target_pose.header.stamp = rospy.Time.now()

            # Pos
            my_goal.target_pose.pose.position.x = userdata.stages[current_stage]["position"]["x"]
            my_goal.target_pose.pose.position.y = userdata.stages[current_stage]["position"]["y"]
            my_goal.target_pose.pose.position.z = 0

            # Orientation
            my_goal.target_pose.pose.orientation.z = userdata.stages[current_stage]["orientation"]["z"]
            my_goal.target_pose.pose.orientation.w = userdata.stages[current_stage]["orientation"]["w"]

            return my_goal

        def navigation_result_cb(userdata, status, result):
            all_status_types = ["pending", "active", "preempted", "succeeded", "aborted", "rejected", "preempting", "recalling", "recalled", "lost"]  # From GoalStatus message definition
            rospy.loginfo(status)
            rospy.loginfo(all_status_types[status])

            # Set new status type (Status of the navigation)
            userdata.status_type = all_status_types[status]

            return all_status_types[status]


        smach.StateMachine.add(
                               "STAGE_UPDATER",
                               StageUpdater(
                                            outcomes=["navigate", "wait_for_request", "completed"],
                                            input_keys=["current_stage", "stages", "status_type"],
                                            output_keys=["current_stage", "status_type"]
                                            ),
                               transitions={
                                            "navigate": "NAVIGATE",
                                            "wait_for_request": "WAIT_FOR_REQUEST",
                                            "completed": "succeeded"
                                            },
                               remapping={"current_stage": "current_stage", "status_type":"status_type"}
                               )
        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState("move_base",
                               MoveBaseAction,
                               goal_cb=navigation_goal_cb,
                               result_cb=navigation_result_cb,
                               input_keys = ["current_stage", "stages"],
                               output_keys = ["status_type"]

                               ),
                               transitions = {
                                            "succeeded": "STAGE_UPDATER",
                                            "aborted": "STAGE_UPDATER",
                                            "preempted": "STAGE_UPDATER"
                                            },
                               remapping={
                                   "current_stage": "current_stage",
                                   "stages": "stages",
                                   "status_type": "status_type"
                               }
                               )

        smach.StateMachine.add("WAIT_FOR_REQUEST",
                               WaitForRequest(
                                            outcomes=["succeeded", "failed"],
                                            input_keys=["status_type"],
                                            output_keys=["status_type"]
                                            ),
                               transitions={
                                            "succeeded":"STAGE_UPDATER",
                                            "failed": "WAIT_FOR_REQUEST"
                                           },
                               remapping={"status_type": "status_type"}
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