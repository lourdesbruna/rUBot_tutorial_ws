#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
	msg = String()
	if data.data == 'Ping':
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		msg.data = 'Pong'
		pub.publish(msg)
		rospy.loginfo(msg.data)
	else:
		msg.data = 'Failed!'
		pub.publish(msg)
		rospy.loginfo(msg.data)

rospy.init_node('pong_node', anonymous=True)
pub = rospy.Publisher("/pong", String, queue_size=10)
sub = rospy.Subscriber("/ping", String, callback)
rospy.spin()