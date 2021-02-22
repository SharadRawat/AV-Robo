#!/usr/bin/python

import numpy as np

class MotorController(object):
	
	def __init__(self, max_speed, max_omega):
		
		# These params are to be tuned.
		
		self.kp = 3
		self.ka = 8
		self.kb = 0
		
		self.max_speed = max_speed
		self.max_omega = max_omega
		
	#def compute_vel(self, state, goal):
		
	#	return vel, omega, is_at_goal
		
