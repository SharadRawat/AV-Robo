import rospy
import roslib
import numpy as np
import cv2
import sys
import yaml

from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Header
from geometry_msgs.msg import (PoseArray,
	PoseStamped,
	Pose,
	Twist,
)


class ROSInterface:
	def __init__(self, t_cam2body):
		self._imu_received = False	
		self._no_detection = True
		self._no_imu = True
		self._imu = None
		self._t = None
		self._R = None
		self._R_cam2body = np.array([[0,0,1,0], [-1,0,0,0], [0,-1,0,0], [0,0,0,1]])
		self._t_cam2body = t_cam2body
		self._R_ultrasonic2body = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
		
		self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		rospy.Subscriber("/imu", Imu, self._imu_callback)
		# For subscribing camera output
		#rospy.Subscriber() 

	def _imu_callback(self, imu):
		self._imu = np.array([[imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.z, imu_header.stamp.to_sec()]]).T
		self._no_imu = False
	
	def get_imu(self):
		if self._no_imu:
			return None
		else:
			return self._imu
	
	#def get_cam_measurements(self):
	
	def command_velocity(self, vx, wz):
		twist_message = Twist()
		twist_message.linear.x = vx
		twist_message.angular.z = wz
		self._pub.publish(twist_message)
		
		
