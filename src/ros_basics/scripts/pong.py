#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

counter = 0

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def pong():
	rospy.init_node('pong_node', anonymous=True)
	pub = rospy.Publisher("/pong", String, queue_size=10)
	sub = rospy.Subscriber("/ping", String, callback)
	rospy.spin()
	
if __name__ == '__main__':
    pong()
