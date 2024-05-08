import rospy
from move_base_msgs.msg import MoveBaseGoal

def navigation_goal_cb(userdata, goal):
    rospy.loginfo(userdata.keys())
    set_location = userdata.set_location

    my_goal = MoveBaseGoal()

    # Header
    my_goal.target_pose.header.frame_id = "map"
    my_goal.target_pose.header.stamp = rospy.Time.now()

    # Pos
    my_goal.target_pose.pose.position.x = userdata.locations[set_location]["position"]["x"]
    my_goal.target_pose.pose.position.y = userdata.locations[set_location]["position"]["y"]
    my_goal.target_pose.pose.position.z = 0

    # Orientation
    my_goal.target_pose.pose.orientation.z = userdata.locations[set_location]["orientation"]["z"]
    my_goal.target_pose.pose.orientation.w = userdata.locations[set_location]["orientation"]["w"]

    return my_goal


def navigation_result_cb(userdata, status, result):
    all_status_types = ["pending", "active", "preempted", "succeeded", "aborted", "rejected", "preempting", "recalling",
                        "recalled", "lost"]  # From GoalStatus message definition
    rospy.loginfo(status)
    rospy.loginfo(all_status_types[status])

    # Set new status type (Status of the navigation)
    userdata.status_type = all_status_types[status]

    return all_status_types[status]