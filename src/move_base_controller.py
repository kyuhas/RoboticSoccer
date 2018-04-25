#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String


def goal_location_callback(goal):

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo(goal)
    move_base.wait_for_server(rospy.Duration(5))
    move_base.send_goal(goal)
    success = move_base.wait_for_result(rospy.Duration(5))

    if not success:
        rospy.loginfo('Failed to reach goal')
        move_base.cancel_all_goals()

    else:
        result_pub.publish('true')
        rospy.loginfo(move_base.get_goal_status_text())

if __name__ == '__main__':

    rospy.init_node('move_base_controller', anonymous=True)

    try:
        result_pub = rospy.Publisher('/move_base_controller_result', String, queue_size=10)
        rospy.Subscriber('/goal_location', MoveBaseGoal, goal_location_callback)
        rospy.spin()

    except rospy.ROSException:
        pass
