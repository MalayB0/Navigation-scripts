#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from actionlib_msgs.msg import GoalID

def cancelGoal():
    pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
    rospy.init_node('cancel_goal', anonymous=True)
    cancel_msg = GoalID()
    pub.publish(cancel_msg)
    rospy.loginfo("Goal canceled")
    
if __name__ == "__main__":
    try:
        flag = input()
        if flag == 99:
            cancelGoal()
            flag = 0
        
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass