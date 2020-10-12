#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
    
if __name__ == "__main__":
    try:
        pub = rospy.Publisher('inputOrder', Int32, queue_size=1)
        rospy.init_node('Order', anonymous=True)
        while not rospy.is_shutdown():
            # 0      : default
            # 1      : stop
            # others : publish destination
            flag = input()

            pub.publish(flag)
            rospy.loginfo("Order published : %d", flag)
            

    except rospy.ROSInterruptException:
        pass