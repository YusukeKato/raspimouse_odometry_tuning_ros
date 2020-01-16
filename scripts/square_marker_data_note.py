#!/usr/bin/env python

# coding: utf-8

from raspimouse_odometry_tuning.msg import CalibrationValue
import pandas as pd
import math, rospy

class DataNote():
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.dn = './experiment_data/'
		self.tn = 'translation/'
		self.rn = 'rotation/'
		self.ex = 'ex_01/'
		self.pub = rospy.Publisher('calibration_value', CalibrationValue, queue_size=1)

		# marker pose [x,y,theta]
		self.marker_pose = [0, 0, 0]

		# mean value
		self.linear_mean = [0, 0, 0]
		self.linear_back_mean = [0, 0, 0]
		self.right_mean = [0, 0, 0]
		self.left_mean = [0, 0, 0]

		# calibration param
		self.linear_L = 1.0
		self.linear_R = 1.0
		self.linear_back_L = 1.0
		self.linear_back_R = 1.0
		self.right_L = 1.0
		self.right_R = 1.0
		self.left_L = 1.0
		self.left_R = 1.0

		# previous calibration param
		self.pre_linear_L = 1.0
		self.pre_linear_R = 1.0
		self.pre_linear_back_L = 1.0
		self.pre_linear_back_R = 1.0
		self.pre_right_L = 1.0
		self.pre_right_R = 1.0
		self.pre_left_L = 1.0
		self.pre_left_R = 1.0

		self.atr = 20.0
		self.atl = 20.0

		self.pre_actual = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
		self.diff_odom = [[2.0, 0, 0], [2.0, 0, 0], [0, 0, 180.0], [0, 0, 180.0]]

		self.first_flag = True

		self.result_L = 0.0
		self.result_angle = 0.0
		self.result_theta = 0.0

	# calc marker pose

	def calc_marker_pose(self, diff_data):
		# calc marker_theta
		marker_theta = 0.0
		for i in range(len(diff_data)):
			if i % 2 == 0: #translation
				x = diff_data[i][0]
				y = diff_data[i][1]
				marker_theta += math.atan2(y, x)
		marker_theta /= 4.0

		# calc marker_xy
		marker_xy = [[0.0],[0.0]]
		rotate_matrix = [[math.cos(marker_theta), -math.sin(marker_theta)],
						 [math.sin(marker_theta),  math.cos(marker_theta)]]
		for i in range(len(diff_data)):
			if i % 2 != 0: #rotation
				x = diff_data[i][0]
				y = diff_data[i][1]
				t = math.radians(diff_data[i][2]) # rad
				L = math.sqrt(x*x+y*y)
				a = math.sin(t/2.0)
				r = 0.0
				if abs(a) > 0.01:
					r = (L/2.0)/a
				marker_xy += rotate_matrix * [[r],[0.0]]
		marker_xy[0][0] /= 4.0
		marker_xy[0][1] /= 4.0

		# update
		self.marker_pose[0] = marker_xy[0][0]
		self.marker_pose[1] = marker_xy[0][1]
		self.marker_pose[2] = marker_theta

	#square----------------------------------------------------------------------
	def diff_data(self, data):
		diff_data = []
		for i in range(1, len(data)):
			x = data[i][0] - data[i-1][0]
			y = data[i][1] - data[i-1][1]
			t = data[i][2] - data[i-1][2]
			a = [x, y, t]
			if a[2] > 180.0:
				a[2] -= 360.0
			elif a[2] < -180.0:
				a[2] += 360.0
			diff_data.append(a)
		return diff_data

	def sum_data(self, data):
		sum_L = 0
		sum_angle = 0
		sum_theta = 0
		for i in range(len(data)):
			if i % 2 == 0: #translation
				sum_L += math.sqrt(data[i][0]**2 + data[i][1]**2)
				sum_angle += data[i][2]
			if i % 2 != 0: #rotation
				sum_theta += data[i][2]
		return sum_L, sum_angle, sum_theta

	def cal_R(self, L, angle):
		return (L/2.0) / math.sin(math.radians(angle)/2.0)

	def cal_dist(self, R, angle):
		return 2.0 * math.pi * R * (angle/360.0)

	def cal_caliVal_translation(self, sum_L, sum_angle):
		target_dist = 0.5
		B = 0.093
		L = sum_L / 4.0
		angle = sum_angle / 4.0
		R = self.cal_R(L, angle)
		actual_dist = self.cal_dist(R, angle)
		
		# by diff dist	
		Ed = abs(target_dist / actual_dist)
		self.linear_R *= Ed
		self.linear_L *= Ed

		if abs(angle) > 1.0:
			# by R and B
			aR = abs(R)
			Er1 = aR / (aR - (B/2.0))	
			Er2 = aR / (aR + (B/2.0))
			if R >= 0:
				self.linear_R *= Er2
				self.linear_L *= Er1
			elif R < 0:
				self.linear_R *= Er1
				self.linear_L *= Er2

	def cal_caliVal_rotation(self, sum_theta):
		target_theta = 90.0
		actual_theta = sum_theta / 4.0
		Er = abs(target_theta / actual_theta)
		# toriaezu
		self.right_R *= Er
		self.right_L *= Er
		self.left_R *= Er
		self.left_L *= Er

	def check_result(self):
		return self.result_L, self.result_angle, self.result_theta

	def squareCaliValue(self, od, tp, mode): #mode, 0:right 1:left
		#diff_od = self.diff_data(od)
		diff_tp = self.diff_data(tp)
		self.calc_marker_pose(diff_tp)
		rospy.loginfo("diff_tp-----------------------------------------")
		for i in range(len(diff_tp)):
			rospy.loginfo(diff_tp[i])
		sum_L, sum_angle, sum_theta = self.sum_data(diff_tp)
		
		# check result -> square_raspimouse_calibration.py
		self.result_L = sum_L
		self.result_angle = sum_angle
		self.result_theta = sum_theta

		self.cal_caliVal_translation(sum_L, sum_angle)
		self.cal_caliVal_rotation(sum_theta)

	def calipublish(self):
		#while not rospy.is_shutdown():
		d = CalibrationValue()
		d.forward_R = self.linear_R
		d.forward_L = self.linear_L
		d.back_R = self.linear_back_R
		d.back_L = self.linear_back_L
		d.right_R = self.right_R
		d.right_L = self.right_L
		d.left_R = self.left_R
		d.left_L = self.left_L
		self.pub.publish(d)

#		rospy.loginfo("----------")
#		rospy.loginfo(self.linear_R)
#		rospy.loginfo(self.linear_L)
#		rospy.loginfo(self.linear_back_R)
#		rospy.loginfo(self.linear_back_L)
#		rospy.loginfo(self.right_R)
#		rospy.loginfo(self.right_L)
#		rospy.loginfo(self.left_R)
#		rospy.loginfo(self.left_L)
		
		#	self.rate.sleep()
		

if __name__ == '__main__':
	dn = './experiment_data/'
	tn = 'translation/'
	rn = 'rotation/'
	ex = 'ex_01/'
	#odom_linear = pd.read_csv(dn+tn+ex+'odom_linear.csv')
	#odom_linear_back = pd.read_csv(dn+tn+ex+'odom_linear_back.csv')
	odom_right = pd.read_csv(dn+rn+ex+'odom_right.csv')
	odom_left = pd.read_csv(dn+rn+ex+'odom_left.csv')
	#true_linear = pd.read_csv(dn+tn+ex+'truePoses_linear.csv')
	#true_linear_back = pd.read_csv(dn+tn+ex+'truePoses_linear_back.csv')
	true_right = pd.read_csv(dn+rn+ex+'truePoses_right.csv')
	true_left = pd.read_csv(dn+rn+ex+'truePoses_left.csv')
	
	#x[m], y[m], theta[deg]
	linear_mean = [0, 0, 0]
	linear_back_mean = [0, 0, 0]
	right_mean = [0, 0, 0]
	left_mean = [0, 0, 0]
	linear_L = 1.0
	linear_R = 1.0
	linear_back_L = 1.0
	linear_back_R = 1.0
	right_L = 1.0
	right_R = 1.0
	left_L = 1.0
	left_R = 1.0
	
	#linear_mean = calcMean(odom_linear, true_linear)
	#linear_back_mean = calcMean(odom_linear_back, true_linear_back)
	right_mean = calcMean(odom_right, true_right)
	left_mean = calcMean(odom_left, true_left)
	
	#print(linear_mean)
	#print(linear_back_mean)
	#print(right_mean)
	#print(left_mean)
	
	#linear_L, linear_R = linearCalib(linear_mean)
	#linear_back_L, linear_back_R = linearCalib(linear_back_mean)
	right_L, right_R = circleCalib(right_mean)
	left_L, left_R = circleCalib(left_mean)
	
	#print(linear_L)
	#print(linear_R)

	rospy.init_node('data_note')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub = rospy.Publisher('calibration_value', CalibrationValue, queue_size=1)
		d = CalibrationValue()
		d.forward_R = linear_R
		d.forward_L = linear_L
		d.back_R = linear_back_R
		d.back_L = linear_back_L
		d.right_R = right_R
		d.right_L = right_L
		d.left_R = left_R
		d.left_L = left_L
		pub.publish(d)
	
		rate.sleep()
