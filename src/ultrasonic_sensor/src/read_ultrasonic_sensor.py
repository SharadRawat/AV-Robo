#!/usr/bin/env python2

import RTIMU
import rospy
import sys
from std_msgs.msg import Float32

def main(args):
	rospy.init_node('ultrasonic_sensor')
	imu_pub = rospy.Publisher('/ultrasonic_sensor_msg/double', Float32, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ola_cabs = rospy.get_time()
		rospy.loginfo(ola_cabs)
		imu_pub.publish(ola_cabs)
		rate.sleep()
		
if __name__ == "__main__":
	try:
		main(sys.argv)
	except rospy.ROSInterruptException: 
		pass
