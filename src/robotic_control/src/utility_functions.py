import numpy as np
import roslib

from tf.transformations import *
from std_msgs.msg import (
	Header,
)

from geometry_msgs.msg import (
	Pose,
	PoseStamped,
	Quaternion
)
	
def get_t_R(pose):
	t = np.transpose(np.matrix([pose.position.x, pose.position.y, pose.position.z, 0]))
	quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
	R_full = quaternion_matrix(quat)
	
	return t, R_full
