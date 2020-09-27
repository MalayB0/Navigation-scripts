import rospy
from actionlib_msgs.msg import GoalID
cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
rospy.init_node('cancel_goal', anonymous=True)
cancel_msg = GoalID()
cancel_pub.publish(cancel_msg)