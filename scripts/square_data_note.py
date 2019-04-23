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

		self.linear_mean = [0, 0, 0]
		self.linear_back_mean = [0, 0, 0]
		self.right_mean = [0, 0, 0]
		self.left_mean = [0, 0, 0]
		self.linear_L = 1.0
		self.linear_R = 1.0
		self.linear_back_L = 1.0
		self.linear_back_R = 1.0
		self.right_L = 1.0
		self.right_R = 1.0
		self.left_L = 1.0
		self.left_R = 1.0

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

#	def calcMean(self, odom, actual):
#		x = y = t = 0
#		x = (odom[1][0] - odom[0][0]) - (actual[1][0] - actual[0][0])
#		y = (odom[1][1] - odom[0][1]) - (actual[1][1] - actual[0][1])
#		tmp = 0
#		tmp = (odom[1][2] - odom[0][2]) - (actual[1][2] - actual[0][2])
#		if tmp < -180: tmp += 360
#		elif tmp > 180: tmp -= 360
#		t = tmp
#
#		ox = odom[1][0] - odom[0][0]
#		oy = odom[1][1] - odom[0][1]
#		ot = odom[1][2] - odom[0][2]
#
#		return [x, y, t], [ox, oy, ot]
#
#	def preGetActual(self, actual):
#		ax = abs(actual[1][0] - actual[0][0])
#		ay = abs(actual[1][1] - actual[0][1])
#		at = abs(actual[1][2] - actual[0][2])
#		#if ax < -180: ax += 360
#		#elif ax > 180: ax -= 360
#		#if ay < -180: ay += 360
#		#elif ay > 180: ay -= 360
#		#if at < -180: at += 360
#		#elif at > 180: at -= 360
#		return [ax, ay, at]
#	
#	def linearCalib(self, mean_forward, mean_back):
#	    dist = 1.5
#	
#	    L_forward = math.sqrt((dist - mean_forward[0])**2 + mean_forward[1]**2)
#	    L_back = math.sqrt((-dist - mean_back[0])**2 + mean_back[1]**2)
#	    R_forward = (L_forward/2.0) / math.sin((mean_forward[2]/180.0*math.pi)/2.0)
#	    R_back = (L_back/2.0) / math.sin((mean_back[2]/180.0*math.pi)/2.0)
#	    enk_forward = 2.0 * math.pi * R_forward * (mean_forward[2]/360.0)
#	    enk_back = 2.0 * math.pi * R_back * (mean_back[2]/360.0)
#	    T = 0.093
#	    
#	    self.linear_R *= abs(dist / enk_forward)
#	    self.linear_L *= abs(dist / enk_forward)
#	    self.linear_back_R *= abs(dist / enk_back)
#	    self.linear_back_L *= abs(dist / enk_back)
#	
#	    if abs(mean_forward[2]) > 1.0:
#			#E1 = (abs(R_forward) + (T/2)) / (abs(R_forward) - (T/2))
#			#E2 = 1.0
#			#E1 = (abs(R_forward) + (T/4)) / (abs(R_forward) - (T/4))
#			#E2 = (abs(R_forward) - (T/4)) / (abs(R_forward) + (T/4))
#			E1 = abs(R_forward)/ (abs(R_forward) - (T/2.0))
#			E2 = abs(R_forward)/ (abs(R_forward) + (T/2.0))
#			if R_forward >= 0:
#				self.linear_R *= E1
#				self.linear_L *= E2
#			elif R_forward < 0:
#				self.linear_L *= E1
#				self.linear_R *= E2
#
#	    if abs(mean_back[2]) > 1.0:
#			#E1 = (abs(R_back) + (T/4)) / (abs(R_back) - (T/4))
#			#E2 = (abs(R_back) - (T/4)) / (abs(R_back) + (T/4))
#			E1 = abs(R_back)/ (abs(R_back) - (T/2.0))
#			E2 = abs(R_back)/ (abs(R_back) + (T/2.0))
#			if R_back >= 0:
#				self.linear_back_L *= E1
#				self.linear_back_R *= E2
#			elif R_back < 0:
#				self.linear_back_R *= E1
#				self.linear_back_L *= E2
#	
#	def circleCalib(self, right_mean, left_mean, actual_r, actual_l):
#		od1 = abs(self.diff_odom[2][2])
#		od2 = abs(self.diff_odom[3][2])
#		print('od1')
#		print(od1)
#		print('od2')
#		print(od2)
#		rate_od1 = 180.0 / od1
#		rate_od2 = 180.0 / od2
#		#if(self.first_flag == True):
#		self.right_L *= 180.0 / (180.0 + right_mean[2]) * rate_od1
#		self.right_R *= 180.0 / (180.0 + right_mean[2]) * rate_od1
#		self.left_L *= 180.0 / (180.0 - left_mean[2]) * rate_od2
#		self.left_R *= 180.0 / (180.0 - left_mean[2]) * rate_od2
#		self.first_flag = False
#		
#		self.atr = actual_r[1][2] - actual_r[0][2]
#		self.atl = actual_l[1][2] - actual_l[0][2]
#		
#		#elif(self.first_flag == False):
#		#	atr = abs(actual_r[1][2] - actual_r[0][2])
#		#	#if atr < -180: atr += 360
#		#	#elif atr > 180: atr -= 360
#		#	print('atr')
#		#	print(atr)
#		#	print('pre_actual_r')
#		#	print(self.pre_actual[2][2])
#		#	atl = abs(actual_l[1][2] - actual_l[0][2])
#		#	#if atl < -180: atl += 360
#		#	#elif atl > 180: atl -= 360
#		#	print('atl')
#		#	print(atl)
#		#	print('pre_actual_l')
#		#	print(self.pre_actual[3][2])
#		#	
#		#	tmp_right_R = self.right_R
#		#	tmp_right_L = self.right_L
#		#	tmp_left_R = self.left_R
#		#	tmp_left_L = self.left_L
#		#	if(abs(atr - 180.0) >= 1.0 and abs(atl - 180.0) >= 1.0):
#		#		a1 = atr - self.pre_actual[2][2]
#		#		a2 = atl - self.pre_actual[3][2]
#		#		#if a1 < -180: a1 += 360
#		#		#elif a1 > 180: a1 -= 360
#		#		#if a2 < -180: a2 += 360
#		#		#elif a2 > 180: a2 -= 360
#		#		rr = (self.right_R - self.pre_right_R) / a1 * (180.0 - atr)
#		#		rl = (self.right_L - self.pre_right_L) / a1 * (180.0 - atr)
#		#		lr = (self.left_R - self.pre_left_R) / a2 * (180.0 - atl)
#		#		ll = (self.left_L - self.pre_left_L) / a2 * (180.0 - atl)
#		#		rr *= rate_od1
#		#		rl *= rate_od1
#		#		lr *= rate_od2
#		#		ll *= rate_od2
#		#		print('pre_right')
#		#		print(self.pre_right_R)
#		#		print('pre_actual')
#		#		print(self.pre_actual[2][2])
#		#		self.right_R = self.right_R + rr
#		#		self.right_L = self.right_L + rl
#		#		self.left_R = self.left_R + lr
#		#		self.left_L = self.left_L + ll
#	#	
#	#		self.pre_right_L = tmp_right_L
#	#		self.pre_right_R = tmp_right_R
#	#		self.pre_left_L = tmp_left_L
#	#		self.pre_left_R = tmp_left_R
#
#	def diffGoal_t(self):
#		L1 = math.sqrt(self.linear_mean[0]**2 + self.linear_mean[1]**2)
#		L2 = math.sqrt(self.linear_back_mean[0]**2 + self.linear_back_mean[1]**2)
#		return L1, L2
#
#	def diffGoal_r(self):
#		R1 = abs(self.atr) - 180.0
#		R2 = abs(self.atl) - 180.0
#		return R1, R2
#
#	def linearCaliValue(self, od, tp, od2, tp2):
#		self.linear_mean, self.diff_odom[0] = self.calcMean(od, tp)
#		self.linear_back_mean, self.diff_odom[1] = self.calcMean(od2, tp2)
#		print(self.linear_mean)
#		print(self.linear_back_mean)
#		self.linearCalib(self.linear_mean, self.linear_back_mean)
#		self.pre_actual[0] = self.preGetActual(tp)
#		self.pre_actual[1] = self.preGetActual(tp2)
#		#self.pre_linear_L = self.linear_L
#		#self.pre_linear_R = self.linear_R
#		#self.pre_linear_back_L = self.linear_back_L
#		#self.pre_linear_back_R = self.linear_back_R
#	
#	def circleCaliValue(self, od, tp, od2, tp2):
#		self.right_mean, self.diff_odom[2] = self.calcMean(od, tp)
#		self.left_mean, self.diff_odom[3] = self.calcMean(od2, tp2)
#		print(self.right_mean)
#		print(self.left_mean)
#		self.circleCalib(self.right_mean, self.left_mean, tp, tp2)
#		print('right_R')
#		print(self.right_R)
#		print('right_L')
#		print(self.right_L)
#		print('left_R')
#		print(self.left_R)
#		print('left_L')
#		print(self.left_L)
#		#self.right_L, self.right_R = self.circleCalib(self.right_mean)
#		#self.left_L, self.left_R = self.circleCalib(self.left_mean)
#		self.pre_actual[2] = self.preGetActual(tp)
#		self.pre_actual[3] = self.preGetActual(tp2)

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
