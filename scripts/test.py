from lab4_pkg.msg import SoftGripperState, SoftGripperCmd
# from geometry_msgs.msg import Twist
import rospy

rospy.init_node('main', anonymous=True)
pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
while not rospy.is_shutdown():
	nextval = input("Set pump motor:")
	msg = SoftGripperCmd(nextval,0)
	# msg = Twist()
	rospy.sleep(1)
	print msg
	pub.publish(msg)