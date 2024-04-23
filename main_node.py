#!/usr/bin/env python

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'], input_keys=["current_stage", "stages"])

    class PointA(smach.State):
        def __init__(self, outcomes, input_keys, output_keys):
            smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        def execute(self, userdata):
            rospy.loginfo("point A")
            userdata.current_stage += 1
            rospy.loginfo(f"Current stage: {userdata.current_stage}")
            return "success"

    with sm:

        sm.userdata.current_stage = 0
        sm.userdata.stages = [
                                {
                                 "position":{"x": 6.14, "y": 3.198230504989624},
                                 "orientation":{"z": -0.71, "w": 0.71}
                                }
                                # Stage(
                                #     stage_number=1,
                                #     position={"x": 6.14, "y": 3.198230504989624},
                                #     orientation={"z": -0.71, "w": 0.71}
                                #     ),
                                # Stage(
                                #     stage_number=2,
                                #     position={"x": 6.14, "y": 8.42},
                                #     orientation={"z": -0.71, "w": 0.71}
                                #     ),
                                # Stage(
                                #     stage_number=3,
                                #     position={"x": 1.956, "y": 2.90412},
                                #     orientation={"z": -0.71, "w": 0.71}
                                #     ),
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

        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState("move_base",
                               MoveBaseAction,
                               goal_cb=test_goal_cb,
                               input_keys = ["current_stage", "stages"]

                               ),
                               transitions = {"succeeded": "POINT_A", "aborted": "NAVIGATE", "preempted": "NAVIGATE"},
                               remapping={
                                   "current_stage": "current_stage",
                                   "stages": "stages"
                               }
                               )
        smach.StateMachine.add(
                               "POINT_A",
                               PointA(
                                    outcomes=["success"],
                                    input_keys=["current_stage"],
                                    output_keys=["current_stage"]
                                    ),
                               transitions={"success": "succeeded"},
                               remapping={"current_stage": "current_stage"}
                               )

        rospy.loginfo(sm.userdata.current_stage)

    sis = IntrospectionServer("my_server", sm, "SM_ROOT")
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()