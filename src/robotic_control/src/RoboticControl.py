#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def sample_publish():
	pub = rospy.Publisher('SampleMsg', String, queue_size=10)
	rospy.init_node('robotic_control', anonymous= True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		string_msg = "Hello there %s" % rospy.get_time()
		rospy.loginfo(string_msg)
		pub.publish(string_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		sample_publish()
	except rospy.ROSInterruptException:
		pass 
