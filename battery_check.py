#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from sensor_msgs.msg import BatteryState


def resultCB(data):
    percentage = data.percentage
    rospy.loginfo(percentage)


if __name__ == "__main__":
    try:    
        rospy.init_node('checkBattery', anonymous=True)
        sub = rospy.Subscriber('battery_state', BatteryState, resultCB, queue_size=10)
        
        rate = rospy.Rate(0.1)

        rospy.spin()
        # goalListX.extend(locationX)
        # goalListY.extend(locationY)

        # while not rospy.is_shutdown():




    except rospy.ROSInterruptException:
        pass