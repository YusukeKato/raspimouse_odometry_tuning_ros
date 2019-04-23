#!/usr/bin/env python
import rospy, tf, math, os
import square_cmdvel
import square_data_note
import pandas as pd
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16 
from std_msgs.msg import Float64
from raspimouse_ros_2.msg import ButtonValues
from raspimouse_odometry_tuning.msg import CalibrationValue

class Calibration():

	def __init__(self):
		self.calibr_flag = 0
		self.button = False

		self.front_R = 1.0
		self.front_L = 1.0
		self.back_R = 1.0
		self.back_L = 1.0
		self.right_R = 1.0
		self.right_L = 1.0
		self.left_R = 1.0
		self.left_L = 1.0

		self.true_pose = []
		self.odom = [] 

		self.true_pose_one = [0, 0, 0]
		self.odom_one = [0, 0, 0]

		self.sub_pose = rospy.Subscriber('true_pose', PoseStamped, self.callbackPose)
		self.sub_odom = rospy.Subscriber('odom', Odometry, self.callbackOdom)
		self.sub_button = rospy.Subscriber('buttons', ButtonValues, self.callbackButton)
		self.sub_flag = rospy.Subscriber('calibr_flag', Int16, self.callbackFlag)

		self.pub_cv = rospy.Publisher('calibration_value', CalibrationValue, queue_size=1)


	def callbackButton(self, message):
		#rospy.loginfo(message)
		self.button = message.rear

	def callbackFlag(self, message):
		#rospy.loginfo(message)
		self.calibr_flag = message.data
		#rospy.loginfo(self.calibr_flag)

	def callbackPose(self, message):
		a = []
		a.append(message.pose.position.x)
		a.append(message.pose.position.y)
		q = message.pose.orientation
		e = self.Quaternion_to_Euler(q)
		theta = e[2] / 3.141592 * 180.0
		if theta < 0: theta = 180 + (180 + theta)
		a.append(theta)#e[2]->EulerAngles.z
		#self.true_pose.append(a)
		self.true_pose_one = a
		
		if len(self.true_pose) >= 10:
			self.true_pose.pop(0)

	def callbackOdom(self, message):
		a = []
		a.append(message.pose.pose.position.x)
		a.append(message.pose.pose.position.y)
		q = message.pose.pose.orientation
		e = self.Quaternion_to_Euler(q)
		theta = e[2] / 3.141592 * 180.0
		if theta < 0: theta = 180 + (180 + theta)
		a.append(theta)#e[2]->EulerAngles.z
		#self.odom.append(a)
		self.odom_one = a

		if len(self.odom) >= 10:
			self.odom.pop(0)

	def Quaternion_to_Euler(self, q):
		e = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
		return e

	def TestCalibration(self):
		tt = square_cmdvel.TestCmdVel()

		count = 0
		times = 5 
		loop_flag = 0

		#for j in range(2):
		j = 0
		tp = []
		tp2 = []
		od = []
		od2 = []
		rospy.loginfo('j')
		rospy.loginfo(j)
		motionFlag = 1

		# Class: cal calibration Value
		dnc = square_data_note.DataNote()

		cali_end = False
		while count < times and cali_end == False:
			if loop_flag == 1: ###START###
				loop_flag = 2
				if motionFlag == 1: #forward
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					tt.test_linear_pub()
				elif motionFlag == 2: #right or left
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					if j == 0:
						tt.test_right_pub()
					elif j == 1:
						tt.test_left_pub()
				elif motionFlag == 3: #forward
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					tt.test_linear_pub()
				elif motionFlag == 4: #right or left
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					if j == 0:
						tt.test_right_pub()
					elif j == 1:
						tt.test_left_pub()
				elif motionFlag == 5: #forward
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					tt.test_linear_pub()
				elif motionFlag == 6: #right or left
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					if j == 0:
						tt.test_right_pub()
					elif j == 1:
						tt.test_left_pub()
				elif motionFlag == 7: #forward
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					tt.test_linear_pub()
				elif motionFlag == 8: #right or left
					motionFlag = 9
					tp.append(self.true_pose_one)
					od.append(self.odom_one)
					if j == 0:
						tt.test_right_pub()
					elif j == 1:
						tt.test_left_pub()

			# update
			elif loop_flag >= 3:
				rospy.loginfo(motionFlag)
				if motionFlag < 8:
					motionFlag += 1
					loop_flag = 0
				elif motionFlag == 9:
					motionFlag = 10
					tp.append(self.true_pose_one)
					od.append(self.odom_one)

			# push button of iPad app
			if self.calibr_flag == 1 and button_flag == True:
				button_flag = False
				loop_flag += 1

			# reset push button
			if self.calibr_flag == 0:
				button_flag = True

			# Cal calibration Value and publish
			if motionFlag >= 10:
				dnc.squareCaliValue(od, tp, j)
				res_L, res_angle, res_theta = dnc.check_result()
				rospy.loginfo("----------")
				rospy.loginfo("diff translation")
				rospy.loginfo(res_L)
				rospy.loginfo(res_angle)
				rospy.loginfo("diff rotation")
				rospy.loginfo(res_theta)
				if abs(res_L - 2.0) < 0.04 and abs(res_angle) < 4.0 and abs(res_theta) < 4.0:
					cali_end = True
				dnc.calipublish()
				del od[:]
				del tp[:]
				del od2[:]
				del tp2[:]
				motionFlag = 1
				loop_flag = 0
				count += 1
				rospy.loginfo("---------------------------")
				rospy.loginfo(count)

		# End Calibration
		print('calibration OK!')
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			dnc.calipublish()
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('raspimouse_calibration')
	rospy.wait_for_service('/motor_on')
	rospy.wait_for_service('/motor_off')
	rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
	rospy.ServiceProxy('/motor_on',Trigger).call()
	
	c = Calibration()
	c.TestCalibration()

	#rospy.spin()
