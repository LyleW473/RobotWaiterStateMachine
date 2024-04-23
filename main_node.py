#!/usr/bin/env python

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=["current_stage", "stages"])

    class StageUpdater(smach.State):
        def __init__(self, outcomes, input_keys, output_keys):
            smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        def execute(self, userdata):
            rospy.loginfo("StageUpdater")
            userdata.current_stage += 1
            rospy.loginfo(f"Current stage: {userdata.current_stage}")

            # Change events depending on whats needed
            if userdata.current_stage == 0:
                return "navigate"
            if userdata.current_stage == 1:
                return "navigate"
            if userdata.current_stage == 2:
                return "navigate"
            if userdata.current_stage == 3:
                return "completed"


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

        def test_goal_cb(userdata, goal):

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

        smach.StateMachine.add(
                               "STAGE_UPDATER",
                               StageUpdater(
                                            outcomes=["navigate", "completed"],
                                            input_keys=["current_stage", "stages"],
                                            output_keys=["current_stage"]
                                            ),
                               transitions={
                                            "navigate": "NAVIGATE",
                                            "completed": "succeeded"
                                            },
                               remapping={"current_stage": "current_stage"}
                               )
        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState("move_base",
                               MoveBaseAction,
                               goal_cb=test_goal_cb,
                               input_keys = ["current_stage", "stages"]

                               ),
                               transitions = {
                                            "succeeded": "STAGE_UPDATER",
                                            "aborted": "NAVIGATE",
                                            "preempted": "NAVIGATE"
                                            },
                               remapping={
                                   "current_stage": "current_stage",
                                   "stages": "stages"
                               }
                               )


        rospy.loginfo(sm.userdata.current_stage)

    sis = IntrospectionServer("my_server", sm, "SM_ROOT")
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()