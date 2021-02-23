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

	def compute_vel(self, state, goal):

		delx = goal[0] - state[0]
		dely = goal[1] - state[1]
		theta = state[2]

		p = np.sqrt((delx ** 2) + (dely ** 2))
		alpha = -theta + np.arctan2(dely, delx)
		beta = -theta - alpha

		v = p * self.kp
		omega = (alpha * self.ka) + (beta * self.kb)
		if np.all(v > self.max_speed):
			v = self.max_speed

		if np.all(omega > self.max_omega):
			omega = self.max_omega

		if np.all(p < 0.15):
			done = True
		else:
			done = False

		vw = (v, omega, done)
		return vw
