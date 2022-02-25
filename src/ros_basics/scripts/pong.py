#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

counter = 0

def callback(msg):
	global counter
	counter += msg.data
	new_msg = String()
	new_msg.data = counter
	pub.publish(new_msg)
	rospy.loginfo("I Publish the counter value: %s", counter)

rospy.init_node('pong_node')
pub = rospy.Publisher("/pong", String, queue_size=10)
sub = rospy.Subscriber("/ping", String, callback)
rospy.spin()
