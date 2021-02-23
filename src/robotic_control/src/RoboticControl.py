#!/usr/bin/env python2

import sys
import rospy
import yaml
import numpy as np
from ROSInterface import ROSInterface

from MotorController import MotorController

class RoboticControl:
	def __init__(self, t_cam2body):
		self.ros_interface = ROSInterface(t_cam2body)

		self.time_init = -1.0
		max_speed = 0.3 # Param
		max_omega = 1.6 # Param
		self.motor_controller = MotorController(max_speed, max_omega)

	def process_measurements(self):
		print("Processing Measurements")
		cam_measurements = self.ros_interface.get_cam_measurements()
		imu_measurements = self.ros_interface.get_imu()
		
		#print("imu measurements are present", imu_measurements)
		
		
		if cam_measurements != None:
			print("Cam measurements: ", cam_measurements)
			state = np.array([0.0, 0.0, 0.0])
			goal = np.array([cam_measurements[0], -cam_measurements[1], cam_measurements[2]])

			vw = self.diff_drive_controller.compute_vel(state, goal)
			print("Computed command vel: ", vw)

			if vw[2] == False:
				self.ros_interface.command_velocity(vw[0], vw[1])

			else:
				self.ros_interface.command_velocity(0, 0)

			return

		else:
			print("No Measurement.")
			return



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
