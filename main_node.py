#!/usr/bin/env python

import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node("main_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    class PointA(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=["success"])
            self.counter = 0
        def execute(self, userdata):
            rospy.loginfo("point A")
            return "success"

    class PointB(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=["success"])
        def execute(self, userdata):
            rospy.loginfo("point B")
            return "success"

    class PointC(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=["success"])
        def execute(self, userdata):
            rospy.loginfo("point C")
            return "success"

    with sm:

        def test_goal_cb(userdata, goal):
            my_goal = MoveBaseGoal()

            # Header
            my_goal.target_pose.header.frame_id = "map"
            my_goal.target_pose.header.stamp = rospy.Time.now()

            # Pos
            my_goal.target_pose.pose.position.x = 6.14
            my_goal.target_pose.pose.position.y = 3.198230504989624
            my_goal.target_pose.pose.position.z = 0

            # Orientation
            my_goal.target_pose.pose.orientation.z = -0.71
            my_goal.target_pose.pose.orientation.w = -0.71

            return my_goal

        smach.StateMachine.add("NAVIGATE",
                               SimpleActionState("move_base", MoveBaseAction, goal_cb=test_goal_cb),
                               transitions={"succeeded": "POINT_A", "aborted": "POINT_B", "preempted": "POINT_C"}
                               )
        smach.StateMachine.add("POINT_A", PointA(), transitions={"success": "succeeded"})
        smach.StateMachine.add("POINT_B", PointB(), transitions={"success": "aborted"})
        smach.StateMachine.add("POINT_C", PointC(), transitions={"success": "aborted"})

    sis = IntrospectionServer("my_server", sm, "SM_ROOT")
    sis.start()
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()