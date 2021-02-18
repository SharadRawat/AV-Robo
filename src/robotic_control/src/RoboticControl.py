#!/usr/bin/env python2

import sys
import rospy
import yaml
import numpy as np
from ROSInterface import ROSInterface

class RoboticControl:
	def __init__(self, t_cam2body):
		self.ros_interface = ROSInterface(t_cam2body)
		
		self.time_init = -1.0
		
	def process_measurements(self):
		#cam_measurements = self.ros_interface.get_cam_measurements()
		imu_measurements = self.ros_interface.get_imu()
		
		if self.time_init == -1.0:
			self.time_init = rospy.get_time()
		self.time_measurement = rospy.get_time()
		diff_time = (self.time_measurement - self.time_init)
		
		if diff_time >=1:
			self.ros_interface.command_velocity(0, 0)
		else:
			self.ros_interface.command_velocity(-5, 0)				

if __name__ == '__main__':
	try:
		rospy.init_node('robotic_control')
		
		param_path = rospy.get_param("~param_path")
		f = open(param_path, 'r')
		params_raw = f.read()
		f.close()
		
		params = yaml.load(params_raw)
		t_cam2body = params['t_cam2body']
		
		robotic_control = RoboticControl(t_cam2body)
		
		rate_of_measurement = rospy.Rate(60)
		while not rospy.is_shutdown():
			robotic_control.process_measurements()
			rate_of_measurement.sleep()
		
		## Once Done, stop the robot
		robotic_control.ros_interface.command_velocity(0, 0)

	except rospy.ROSInterruptException:
		pass 
