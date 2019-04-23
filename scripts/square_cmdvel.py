#!/usr/bin/env python
import rospy, tf, sys, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int16

class TestCmdVel():

	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
		#self.pub_flag = rospy.Publisher('/calibr_flag_2', Int16, queue_size=1)
		self.rate_value = 120 
		self.rate = rospy.Rate(self.rate_value)

		self.p_x = 0
		self.pp_x = 0
		self.p_y = 0
		self.pp_y = 0
		self.r_z = 0
		self.rr_z = 0

		#self.flag = False

		self.vel_zero = Twist()

		# mode val
		self.m = 0

	def Quaternion_to_Euler(self, q):
		self.e = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
		return self.e

	def callback(self, message):
		self.p_x = message.pose.pose.position.x
		self.p_y = message.pose.pose.position.y
		self.q = self.Quaternion_to_Euler(message.pose.pose.orientation)
		self.r_z = self.q[2] / 3.1415 * 180
		if self.r_z < 0: self.r_z = 180 + (180 + self.r_z) #-180->180, 0->360
		#rospy.loginfo(self.p_x)
		#rospy.loginfo('self.r_z')
		#rospy.loginfo(self.r_z)

	def test_linear_pub(self):
		target = 0.5
		speed = 0.2
		target_time = target / speed

		self.test_vel = Twist()
		self.test_vel.linear.x = speed
		self.test_vel.angular.z = 0
	
		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= 2.0:
			self.pp_x = self.p_x
			time_e = time.time()
		
		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= target_time:
			self.pub.publish(self.test_vel)
			time_e = time.time()
			#self.rate.sleep()
		self.pub.publish(self.vel_zero)

#		rospy.loginfo("----------")
#		rospy.loginfo("time[s]")
#		rospy.loginfo(time_e - time_s)
#		rospy.loginfo("distance[m]")
#		rospy.loginfo(self.p_x - self.pp_x)
#		rospy.loginfo("position")
#		rospy.loginfo("X")
#		rospy.loginfo(self.p_x)
#		rospy.loginfo("Y")
#		rospy.loginfo(self.p_y)
#		rospy.loginfo("RZ")
#		rospy.loginfo(self.r_z)


	def test_linear_back_pub(self):
		target = 0.5
		speed = -0.2
		target_time = target / speed

		self.test_vel = Twist()
		self.test_vel.linear.x = speed
		self.test_vel.angular.z = 0
	
		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= 2.0:
			self.pp_x = self.p_x
			time_e = time.time()
		
		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= target_time:
			self.pub.publish(self.test_vel)
			time_e = time.time()
			#self.rate.sleep()
		self.pub.publish(self.vel_zero)

#		rospy.loginfo("----------")
#		rospy.loginfo("time[s]")
#		rospy.loginfo(time_e - time_s)
#		rospy.loginfo("distance[m]")
#		rospy.loginfo(self.p_x - self.pp_x)
#		rospy.loginfo("position")
#		rospy.loginfo("X")
#		rospy.loginfo(self.p_x)
#		rospy.loginfo("Y")
#		rospy.loginfo(self.p_y)
#		rospy.loginfo("RZ")
#		rospy.loginfo(self.r_z)

	def test_left_pub(self):
		target = 90.0
		speed = 90.0
		target_time = target / speed

		self.test_vel = Twist()
		self.test_vel.linear.x = 0
		self.test_vel.angular.z = speed * 3.1415 / 180.0

		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= 2.0:
			self.rr_z = self.r_z
			time_e = time.time()

		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= target_time:
			self.pub.publish(self.test_vel)
			time_e = time.time()
			#self.rate.sleep()
		self.pub.publish(self.vel_zero)

#		rospy.loginfo("----------")
#		rospy.loginfo("time[s]")
#		rospy.loginfo(time_e - time_s)
#		rospy.loginfo("rotation[deg]")
#		rospy.loginfo(self.r_z - self.rr_z)
#		rospy.loginfo("position")
#		rospy.loginfo("X")
#		rospy.loginfo(self.p_x)
#		rospy.loginfo("Y")
#		rospy.loginfo(self.p_y)
#		rospy.loginfo("RZ")
#		rospy.loginfo(self.r_z)

	def test_right_pub(self):
		target = -90.0
		speed = -90.0
		target_time = target / speed

		self.test_vel = Twist()
		self.test_vel.linear.x = 0
		self.test_vel.angular.z = speed * 3.1415 / 180.0

		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= 2.0:
			self.rr_z = self.r_z
			time_e = time.time()

		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= target_time:
			self.pub.publish(self.test_vel)
			time_e = time.time()
			#self.rate.sleep()
		self.pub.publish(self.vel_zero)

#		rospy.loginfo("----------")
#		rospy.loginfo("time[s]")
#		rospy.loginfo(time_e - time_s)
#		rospy.loginfo("rotation[deg]")
#		rospy.loginfo(self.r_z - self.rr_z)
#		rospy.loginfo("position")
#		rospy.loginfo("X")
#		rospy.loginfo(self.p_x)
#		rospy.loginfo("Y")
#		rospy.loginfo(self.p_y)
#		rospy.loginfo("RZ")
#		rospy.loginfo(self.r_z)

	def get_mode(self):
		self.m = rospy.get_param('motion_mode', 0)			

	def time_count(self, target_time):
		time_s = time.time()
		time_e = time.time()
		while time_e - time_s <= target_time:
			time_e = time.time()
			#self.rate.sleep()
		
	
if __name__ == '__main__':
	rospy.init_node('test_cmdvel_publisher')
	rospy.wait_for_service('/motor_on')
	rospy.wait_for_service('/motor_off')
	rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
	rospy.ServiceProxy('/motor_on',Trigger).call()
	
	#args = sys.argv
	tt = TestCmdVel()
	tt.get_mode()

	# right
	if tt.m == 1:
		for i in range(4):
			tt.test_linear_pub()
			tt.time_count(2.0)
			tt.test_right_pub()
			tt.time_count(2.0)

	# left
	elif tt.m == 2:
		for i in range(4):
			tt.test_linear_pub()
			tt.time_count(2.0)
			tt.test_left_pub()
			tt.time_count(2.0)

	else:
		rospy.loginfo("motion mode null")

#	if tt.m == 1:
#		tt.test_linear_pub()
#	elif tt.m == 2:
#		tt.test_linear_back_pub()
#	elif tt.m == 3:
#		tt.test_left_pub()
#	elif tt.m == 4:
#		tt.test_right_pub()
#	else:
#		print('null')
	#rospy.spin()
