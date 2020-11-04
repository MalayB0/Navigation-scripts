#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import string
import math
import time
import sys
import subprocess



DEFAULT = 0
STOP = -1
LOW_BATTERY = -2


#from std_msgs.msg import String

from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
goalId = 0


def pub_Twist(angle):
    chk = Twist()

    chk.linear.x = 0
    chk.angular.z = angle

    est.publish(chk)

def locationCheck():
    
    pub_Twist(2.5)
    rospy.loginfo("Estimate location...")
    
    rospy.sleep(5.0)

    pub_Twist(0)
    rospy.loginfo("Estimate finished!")
    
    
    #subprocess.call("rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{angular: {x: 0.0,y: 0.0,z: 2.5}}'", shell=True)
    #subprocess.call("rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{angular: {x: 0.0,y: 0.0,z: 0.0}}'", shell=True)
    
    


def goTo(index):
    goalMsg.header.stamp = rospy.Time.now()

    if flag == LOW_BATTERY:
        goalMsg.pose.position.x = 0
        goalMsg.pose.position.y = 0
    else:
        goalMsg.pose.position.x = goalListX[index]
        goalMsg.pose.position.y = goalListY[index]
        
    
    pub.publish(goalMsg)
    rospy.loginfo("Initial goal published! Goal ID is: %d", index)

def stop():
    rospy.loginfo("Cancel goal")
    subprocess.call('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}', shell=True)

def init(goalListX, goalListY, retry, map_frame):
    global goalId
    goalMsg.header.frame_id = map_frame
    goalMsg.pose.orientation.z = 0.0
    goalMsg.pose.orientation.w = 1.0
    
    time.sleep(1)

    goTo(goalId)

    goalId = goalId + 1 # goalId : 0 -> 1
    
def resultCB(data):
    global flag
    global goalId

    if flag == 0:
        
        if data.status.status == 3: # reached
            goTo(goalId)
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0 
    

    elif flag > 0:
        resultMSG = Int32()    
        resultMSG = 2**flag
        rst.publish(resultMSG)


def orderCB(data):
    global flag
    global nowX
    global nowY
    flag = data.data
    rospy.loginfo("Received flag : %d", flag)
    checkDist = 10

##
    if flag > 0:
        calcFlag = flag

        for i in range(8,-1,-1):
            if calcFlag == 2**i:
                flag = i
                break

##

    if flag == DEFAULT:
        rospy.loginfo("Location of robot : (%f, %f)", nowX, nowY)
        # find nearest point
        for i in range(len(goalListY)):
            rangeI = math.sqrt((nowX - goalListX[i])**2 + (nowY - goalListY[i])**2)
            rospy.loginfo("Distance to point %d : %f", i,rangeI)
            if rangeI < checkDist:
                checkDist = rangeI
                nearestPoint = i
        
        rospy.loginfo("Nearest point : %d ", nearestPoint+1)
        goalId = nearestPoint
        goTo(goalId)



    elif flag == STOP:
        stop()

    elif flag == LOW_BATTERY:
        goTo(LOW_BATTERY)

    # 
    elif flag > 1:
        
        goTo(flag-1)


    else:
        rospy.loginfo("Invalid flag")



    #else:
    #    rospy.loginfo("Wrong input : %d ", flag)
    #    pass

def feedbackCB(data): 
    global goalId
    global nowX
    global nowY
    # calculate distance to destination
    nowX = data.feedback.base_position.pose.position.x
    nowY = data.feedback.base_position.pose.position.y
    distX = nowX - goalListX[goalId-1]
    distY = nowY - goalListY[goalId-1]
    dist = math.sqrt(distX*distX + distY*distY)
    if flag == 0:
        if dist < 0.08: # If the distance is shorter than 7cm, publish next goal
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0            

            goTo(goalId-1)
            
            rospy.loginfo("flag : %d",flag)           



if __name__ == "__main__":
    try:    
        goalId = 0
        flag = 0 # 0 : default
        nowX = 0; nowY = 0
        # ROS Init  
        rospy.init_node('userControl', anonymous=True)
        sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, resultCB, queue_size=10)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        arv = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, feedbackCB, queue_size=10)
        ord = rospy.Subscriber('inputOrder', Int32, orderCB, queue_size=1)
        est = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rst = rospy.Publisher('r_flag', Int32, queue_size = 10)

        rate = rospy.Rate(10)

        #locationCheck()

        goalMsg = PoseStamped()
        


        # Get params
        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 
        goalListX = [0.00, 0.85, 0.85, 0.00, 0.00,-0.85,-0.85, 0.00]
        goalListY = [0.75, 0.75,-0.75,-0.75, 0.75, 0.75,-0.75,-0.75]        
        
        orderListX = []
        orderListY = []

        # goalListX.extend(locationX)
        # goalListY.extend(locationY)

        while not rospy.is_shutdown():
            if flag == 0:

                if (len(goalListX) == len(goalListY)) & (len(goalListY) >=2):          
                    # Constract MultiGoals Obj
                    rospy.loginfo("Starting robot...")
                    init(goalListX, goalListY, retry, map_frame)
                    rospy.spin()

                else:
                    rospy.loginfo("Lengths of goal lists are not the same")

    except rospy.ROSInterruptException:
        pass