#!/usr/bin/env python2

import RTIMU
import rospy
import roslib
import sys
import numpy as np
import numpy.matlib

from std_msgs.msg import (Header,
)

from sensor_msgs.msg import (Imu,
)

from geometry_msgs.msg import (Vector3,
)

class ImuReader(object):
	def __init__(self):
		calib_path = rospy.get_param("~calib_path")
		self._imu_settings = RTIMU.Settings(calib_path)
		self._imu = RTIMU.RTIMU(self._imu_settings)
		self._imu.IMUInit()
		poll_interval = self._imu.IMUGetPollInterval()
		self._rate = 1000./poll_interval
		self._GRAVITY = 9.81
	
	def read_from_imu(self):
		self._imu.IMURead()
		readout = self._imu.getIMUData()
		accel = readout['accel']
		omega = readout['gyro']
		bearing = readout['compass']
		
		return accel, omega, bearing
	
	def fill_imu_msg(self, accel, omega):
		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = 'imu'
		imu_msg.orientation_covariance[0] = -1
		imu_msg.angular_velocity = Vector3(omega[0], omega[1], omega[2])
		imu_msg.linear_acceleration = Vector3(accel[0]*self._GRAVITY , accel[1]*self._GRAVITY, accel[2]*self._GRAVITY)
		
		#rospy.loginfo(imu_msg.linear_acceleration)
		return imu_msg
		
	

def main(args):
	rospy.init_node('imu')
	imu_reader = ImuReader()
	imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
	rate = rospy.Rate(imu_reader._rate)
	while not rospy.is_shutdown():
		(accel, omega, bearing) = imu_reader.read_from_imu()
		imu_msg = imu_reader.fill_imu_msg(accel, omega)
		imu_pub.publish(imu_msg)
		rate.sleep()
		
if __name__ == "__main__":
	try:
		main(sys.argv)
	except rospy.ROSInterruptException: 
		pass
