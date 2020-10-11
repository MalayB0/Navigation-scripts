#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

PI = 3.1415926535897

class movement :

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)
        self.pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.move = Twist()

    def publish_vel(self):
        self.pub_move.publish(self.move)

    def move_forward(self):        
        self.move.linear.x=1
        self.move.angular.z=0.0

    def move_backward(self):      
        self.move.linear.x=-1
        self.move.angular.z=0.0

    def stop(self):        
        self.move.linear.x=0
        self.move.angular.z=0.0  


if __name__ == "__main__":

    mov = movement()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown() :

        movement = raw_input('Enter desired movement: ')

        if movement == 'forward':
            mov.move_forward()

        if movement == 'backward':
            mov.move_backward()

        if movement == 'stop':
            mov.stop()

        mov.publish_vel()
        rate.sleep()