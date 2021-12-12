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
from sensor_msgs.msg import BatteryState

goalId = 0


# 초기설정
def init(goalListX, goalListY, retry, map_frame): 
    global goalId
    goalMsg.header.frame_id = map_frame
    goalMsg.pose.orientation.z = 0.0
    goalMsg.pose.orientation.w = 1.0
    
    time.sleep(1)

    setDestination(goalId)

    goalId = goalId + 1 # goalId : 0 -> 1




def pub_Twist(angle): 
    chk = Twist()

    chk.linear.x = 0
    chk.angular.z = angle

    est.publish(chk)


# 초기 위치 교정
def locationCheck(): 
    
    pub_Twist(2.5)
    rospy.loginfo("Estimate location...")
    
    rospy.sleep(5.0)

    pub_Twist(0)
    rospy.loginfo("Estimate finished!")
    
    

# 목적지로 이동
def setDestination(index): 
    goalMsg.header.stamp = rospy.Time.now()

    if flag == LOW_BATTERY:
        goalMsg.pose.position.x = 0
        goalMsg.pose.position.y = 0
        rospy.loginfo("Please charge battery")           

    else:
        goalMsg.pose.position.x = goalListX[index]
        goalMsg.pose.position.y = goalListY[index]
        
    
    pub.publish(goalMsg)
    rospy.loginfo("Initial goal published! Goal ID is: %d", index)


# 로봇 정지
def stop(): 
    rospy.loginfo("Cancel goal")
    subprocess.call('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}', shell=True)




# 목적지에 도착했을 경우
# Deafult 상태일 경우 다음 목적지 publish
# Default 상태가 아닐 경우 음성 코드에 도착메세지 publish
def resultCB(data):
    global flag
    global goalId

    if flag == 0:
        
        if data.status.status == 3: # reached
            setDestination(goalId)
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0 
    

    elif flag > 0:
        if data.status.status == 3: # reached
            resultMSG = Int32()    
            resultMSG = 2**flag
            rst.publish(resultMSG)


# 상태 변화 flag를 받았을 경우
# ex : 목적지 정보, 완료메세지, 정지메세지, 배터리 메세지 등..
def orderCB(data):
    global flag
    global nowX
    global nowY
    global goalId

    flag = data.data
    rospy.loginfo("Received flag : %d", flag)
    checkDist = 10

    ##  수신된 정보 가공
    if flag > 0:
        calcFlag = flag

        for i in range(8,-1,-1):
            if calcFlag == 2**i:
                flag = i
                break

    ##  DEFAULT 상태로 돌아가는 메세지일 경우
    if flag == DEFAULT:
        rospy.loginfo("Location of robot : (%f, %f)", nowX, nowY)
        # find nearest point
        for i in range(len(goalListY)):
            rangeI = math.sqrt((nowX - goalListX[i])**2 + (nowY - goalListY[i])**2)
            rospy.loginfo("Distance to point %d : %f", i,rangeI)
            if  checkDist > rangeI:
                checkDist = rangeI
                nearestPoint = i
        # 현재 위치로부터 가장 가까운 지점을 찾고, 거기서부터 다시 배회 시작
        rospy.loginfo("Nearest point : %d ", nearestPoint+1)
        goalId = nearestPoint
        setDestination(goalId)


    ##  정지 메세지를 받았을 경우 정지
    elif flag == STOP:
        stop()

    ##  배터리 부족 메세지를 받았을 경우 충전소로 이동
    elif flag == LOW_BATTERY:
        setDestination(LOW_BATTERY)

    ##  목적지 정보를 받았을 경우 해당 목적지로 이동
    elif flag > 0:
        setDestination(flag-1)

    ##  그외 : 잘못된 flag
    else:
        rospy.loginfo("Invalid flag")


# 현재 위치를 파악하며 진행
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
    if flag == DEFAULT:
        if dist < 0.08: # If the distance is shorter than 7cm, publish next goal
            if goalId < (len(goalListX)-1):
                goalId = goalId + 1
            else:
                goalId = 0            

            setDestination(goalId-1)
            
            rospy.loginfo("flag : %d",flag)           


# 배터리 부족할 시
def batteryCB(data):
    global flag
    global Volt
    Volt = data.voltage
    if Volt < 11.1:
        flag = LOW_BATTERY
        setDestination(LOW_BATTERY)



# 기본 로컬파라미터 정보, 메세지 선언 등
if __name__ == "__main__":
    try:    
        goalId = 0
        flag = 0 # 0 : default
        nowX = 0; nowY = 0
        volt = 11.5
        # ROS Init  
        rospy.init_node('userControl', anonymous=True)
        sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, resultCB, queue_size=10)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        arv = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, feedbackCB, queue_size=10)
        ord = rospy.Subscriber('inputOrder', Int32, orderCB, queue_size=1)
        est = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rst = rospy.Publisher('r_flag', Int32, queue_size = 10)
        btr = rospy.Subscriber('battery_state', BatteryState, batteryCB, queue_size=10)
        rate = rospy.Rate(10)

        #locationCheck()

        goalMsg = PoseStamped()
        


        # Get params
        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 
        #goalListX = [0.00, 0.85, 0.85, 0.00, 0.00,-0.85,-0.85, 0.00]
        #goalListY = [0.75, 0.75,-0.75,-0.75, 0.75, 0.75,-0.75,-0.75] 

        
        #goalListX = [-0.89, -0.59, 1.36, 0.96, -0.89, -1.39, 0.61, 0.96]
        #goalListY = [0.45, 1.33, 0.62, -0.31, 0.45, -0.64, -1.36, -0.31] 

        goalListX = [-1.0, -0.93, 1.08, 1.0, -1.0, -1.22, 0.95, 1.0]
        goalListY = [0.11, 1.14, 0.87, -0.1, 0.11, -0.96, -1.23, -0.1] 

        
        
        # 원점 : 0,0


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