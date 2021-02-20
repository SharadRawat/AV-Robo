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

from apriltags_ros.msg import (
	AprilTagDetectionArray,
	AprilTagDetection
)

from utility_functions import *

class ROSInterface:
	def __init__(self, t_cam2body):
		self._imu_received = False	
		self._no_detection = True
		self._no_imu = True
		self._imu = None
		self._t = None
		self._R = None
		self._angle = None
		self._R_cam2body = np.array([[0,0,1,0], [-1,0,0,0], [0,-1,0,0], [0,0,0,1]])
		self._t_cam2body = t_cam2body
		self._R_ultrasonic2body = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
		self._R_tag2bot = np.array([[0,-1,0,0], [0,0,1,0], [-1,0,0,0], [0,0,0,1]])
		
		self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		rospy.Subscriber("/imu", Imu, self._imu_callback)
		rospy.Subscriber("/camera/tag_detections", AprilTagDetectionArray, self._tag_pose_callback ) 

	def _tag_pose_callback(self, pose_array):
		if(len(pose_array.detections)==0):
			return
		
		(self._t, self._R) = get_t_R(pose_array.detections[0].pose.pose)
		self._angle = -np.arctan2(-self._R[2,0], np.sqrt(self._R[2,0]**2+self._R[2,2]**2))
		
		if (math.isnan(self._angle)):
			return
		
		self._R = np.dot(np.dot(self._R_cam2body, self._R), self._R_tag2bot)
		self._t = np.dot(self._R_cam2body, self._t) + self._t_cam2body
		self._marker_num = pose_array.detections[0].id
		self._no_detection = False

	
	def _imu_callback(self, imu):
		self._imu = np.array([[imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.z, imu.header.stamp.to_sec()]]).T
		self._no_imu = False
	
	def get_imu(self):
		if self._no_imu:
			return None
		else:
			return self._imu
	
	def get_cam_measurements(self):
		if self._no_detection:
			return
		
		self._no_detection = True
		dx = self._t[0,0]
		dy = self._t[1,0]
		
		return [[dx,dy, self._angle, self._marker_num]]
			
	
	def command_velocity(self, vx, wz):
		twist_message = Twist()
		twist_message.linear.x = vx
		twist_message.angular.z = wz
		self._pub.publish(twist_message)
		
		
