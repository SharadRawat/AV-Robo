#!/usr/bin/env python2

import RTIMU
import rospy
import sys
from std_msgs.msg import String

def main(args):
	rospy.init_node('imu')
	imu_pub = rospy.Publisher('imu', String, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ola_cabs = "Printing rubbish at %s" % rospy.get_time()
		rospy.loginfo(ola_cabs)
		imu_pub.publish(ola_cabs)
		rate.sleep()
		
if __name__ == "__main__":
	try:
		main(sys.argv)
	except rospy.ROSInterruptException: 
		pass
