#!/usr/bin/env python2

import rospy
import sys
import serial
import statistics
from std_msgs.msg import Float32

ser = serial.Serial('/dev/ttyUSB0',9600)


def distance():
    s = [0,1]
    read_serial=ser.readline().strip()
    values = read_serial.decode('ascii').split(',')
    a = [float(s) for s in values]
    return a

def main(args):
	rospy.init_node('ultrasonic_sensor')
	imu_pub = rospy.Publisher('/ultrasonic_sensor_msg', Float32, queue_size=10)
	rate = rospy.Rate(10)
	dist = []
	count = 0
	while not rospy.is_shutdown():
	    if count >= 20:
		dist = dist[len(dist)-10:]
	    dist += distance()
	    distance_msg = statistics.mean(dist)
	    #rospy.loginfo(distance_msg)
	    imu_pub.publish(distance_msg)
	    count +=1
	    rate.sleep()
		
if __name__ == "__main__":
	try:
		main(sys.argv)
	except rospy.ROSInterruptException: 
		pass
