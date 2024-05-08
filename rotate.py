import rospy
from smach import State
from geometry_msgs.msg import Twist

class RotateState(State):
    def __init__(self, outcomes, input_keys, output_keys):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def execute(self, userdata):

        # Rotate state used to localise robot (rotate for 2.5 seconds)
        if userdata.rotate_to_localise == True:
            old_time = rospy.get_time()
            while rospy.get_time() - old_time < 2.5:
                self.rotate_inplace(rotate_speed=1)

            userdata.rotate_to_localise = False
            userdata.status_type = "succeeded"
        else:
            self.rotate_inplace(rotate_speed=5)
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