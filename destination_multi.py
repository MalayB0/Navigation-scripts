#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import string
import math
import time
import sys
import subprocess

#from std_msgs.msg import String

from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import Int32

goalId = 0

def findNearestPoint(List):
    checkDist = 10
    # find nearest point
    for i in range(len(List)):
        rangeI = math.sqrt((nowX - ListX[i])**2 + (nowY - ListY[i])**2)
        rospy.loginfo("Distance to point %d : %f", i,rangeI)
        if rangeI < checkDist:
            checkDist = rangeI
            nearestPoint = i
    return nearestPoint




def setDestination(X,Y):
    goalMsg.header.stamp = rospy.Time.now()
    goalMsg.pose.position.x = X
    goalMsg.pose.position.y = Y
    pub.publish(goalMsg)
    rospy.loginfo("Initial goal published! Location is: %f,%f", X,Y)

def stop():
    rospy.loginfo("Cancel goal")
    subprocess.call('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}', shell=True)



def init(goalListX, goalListY, retry, map_frame):
    global goalId
    goalMsg.header.frame_id = map_frame
    goalMsg.pose.orientation.z = 0.0
    goalMsg.pose.orientation.w = 1.0
    
    time.sleep(1)

    setDestination(goalListX[goalId], goalListY[goalId])

    goalId = goalId + 1 # goalId : 0 -> 1
    


def resultCB(data):
    global flag

    if flag == 0:
        global goalId
        if data.status.status == 3: # reached
            setDestination(goalListX[goalId], goalListY[goalId])
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0 



    elif (flag > 1) & (flag%2 == 0):
        global orderId
        # when guide finished, finished = 1
        finished = 0


        if data.status.status == 3: # reached
            # wait 3 seconds
            time.sleep(3)

            setDestination(orderListX[orderId], orderListY[orderId])

            if orderId < (len(orderListX)-1):
                orderId = orderId + 1
            
            # end guide
            elif orderId == len(orderListX)-1:
                finished = 1
                rospy.loginfo("finished")



def orderCB(data):
    global flag
    global nowX
    global nowY
    flag = data.data
    rospy.loginfo("Received flag : %d", flag)
    checkDist = 10


    if flag == 0:
        rospy.loginfo("Recent Location : (%f, %f)", nowX, nowY)
        
        # find nearest point
        for i in range(len(goalListY)):
            rangeI = math.sqrt((nowX - goalListX[i])**2 + (nowY - goalListY[i])**2)
            rospy.loginfo("Distance to point %d : %f", i,rangeI)
            if rangeI < checkDist:
                checkDist = rangeI
                nearestPoint = i
        
        rospy.loginfo("Nearest point : %d ", nearestPoint+1)
        goalId = nearestPoint
        setDestination(goalListX[goalId], goalListY[goalId])



    elif flag == 1:
        stop()

    
    # 
    elif (flag > 1) & (flag%2 == 0):
        calcFlag = flag
        global orderListX
        global orderListY
        global orderId
        global nearestPoint
        # 
        orderId = []
        orderListX = []
        orderListY = []
        
        for i in range(8,0,-1):
            if (calcFlag - 2**i) > -1:
                calcFlag = calcFlag - 2**i
                orderList.append(i)
                orderListX.append(goalListX[i-1])
                orderListY.append(goalListY[i-1])
        rospy.loginfo("orderList :" , orderList)
        rospy.loginfo("orderListX :", orderListX)
        rospy.loginfo("orderListY :", orderListY)
        rospy.loginfo("Start navigate")
        rospy.loginfo("Location of robot : [%.2f, %.2f]", nowX, nowY)
  
        nearestPoint = findNearestPoint(orderList)

        rospy.loginfo("Nearest point : %d ", nearestPoint+1)

        setDestination(orderListX[nearestPoint],orderListY[nearestPoint])
        orderList.remove(orderList[nearestPoint])
        




    else:
        roapy.loginfo("Invalid flag")



    #else:
    #    rospy.loginfo("Wrong input : %d ", flag)
    #    pass


def feedbackCB(data):
    global goalId
    global nowX
    global nowY
    global nearestPoint
    global flag
    # calculating distance to destination
    nowX = data.feedback.base_position.pose.position.x
    nowY = data.feedback.base_position.pose.position.y

    if flag == 0:
        distX = nowX - goalListX[goalId-1]
        distY = nowY - goalListY[goalId-1]
        dist = math.sqrt(distX*distX + distY*distY)
        if dist < 0.08: # If the distance is shorter than 7cm, publish next goal
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0            


            #goTo(goalId-1)
            setDestination(goalListX[goalId-1], goalListY[goalId-1])
            rospy.loginfo("flag : %d",flag)           

    elif flag > 1:
        distX = nowX - orderListX[nearestPoint]
        distY = nowY - orderListY[nearestPoint]
        dist = math.sqrt(distX*distX + distY*distY)   
        #rospy.loginfo("계산하고있음 : %f",dist)         
        if (flag%2 == 0) & (dist < 0.08):
            
            if len(orderList) > 0:
                
                nearestPoint = findNearestPoint(orderList)

                rospy.loginfo("Next nearest point : %d (%d)", nearestPoint, orderList[nearestPoint])

                setDestination(orderListX[nearestPoint],orderListY[nearestPoint])
                orderList.remove(orderList[nearestPoint])

            elif len(orderList) == 0:
                rospy.loginfo("Navigate finished. change to default mode.")
                stop()
                time.sleep(5)
                flag = 0



    else :
        rospy.loginfo("Wrong flag")


if __name__ == "__main__":
    try:    
        goalId = 0
        flag = 0 # 0 : default
        orderId = 0
        nowX = 0; nowY = 0
        nearestPoint = -1
        # ROS Init  
        rospy.init_node('userControl', anonymous=True)
        sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, resultCB, queue_size=10)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        arv = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, feedbackCB, queue_size=10)
        ord = rospy.Subscriber('inputOrder', Int32, orderCB, queue_size=1)

        goalMsg = PoseStamped()
        
        # Get params
        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 
        goalListX = [0.00, 0.66, 0.66, 0.00, 0.00,-0.66,-0.66, 0.00]
        goalListY = [0.66, 0.66,-0.66,-0.66, 0.66, 0.66,-0.66,-0.66]        
        orderList = []    
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
                    rospy.errinfo("Lengths of goal lists are not the same")

    except rospy.ROSInterruptException:
        pass