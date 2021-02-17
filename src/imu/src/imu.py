import RTIMU
from std_msgs import String

def main(args):
	rospy.init_node('imu')
	imu_pub = rospy.Publisher('imu', String, queue_size=10)

if __name__ == "__main__":
	try:
		main(sys.argv)
	except rospy.ROSInterruptException: 
		pass
