#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import roslib
import sys, time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from main2 import *
from hexapodo.msg import Num


tau_1 = 0.1
tau_2 = 0.1

#w_11 = 0.5
#w_22 = 0.7
w_11 = 0.1
w_22 = 0.1
w_21 = 0.6
w_12 = 1

w_41 = 3
w_32 = 3
w_31 = 3
w_42 = 3


w_61 = 10
w_72 = 10

w_53 = 2
w_54 = 2

w_56 = 15
w_57 = 15

h = 0.02
t = 0
k = 0
L = 0
X_1 = np.zeros((2))
X_2 = np.zeros((2))
X_3 = np.zeros((2))
X_4 = np.zeros((2))
X_5 = np.zeros((2))
X_6 = np.zeros((2))
X_7 = np.zeros((2))

x1_b = 0
x2_b = 4
y1_b = 2.25
y2_b = 1.25

m_b = (y2_b - y1_b)/(x2_b - x1_b)
b_b = y1_b - m_b * x1_b


x1_r = 0
x2_r = 4
y1_r = 3.5
y2_r = 2

m_r = (y2_r - y1_r)/(x2_r - x1_r)
b_r = y1_r - m_r * x1_r

x1_i = 4
y1_i = 0.18
x2_i = 8
y2_i = 0.11
m_i = (y2_i - y1_i)/(x2_i - x1_i)
b_i = y1_i - m_i*x1_i

def calc_angle(lb, lr):

	angle_b = []
	angle_r = []
	if len(lb) != 0:
		angle_b = [np.arctan2(lb[0]-400,489)]
		angle_b = [np.rad2deg(angle_b)]

	if len(lr) != 0:
		angle_r = [np.arctan2(lr[0]-400,489)]
		angle_r = [np.rad2deg(angle_r)]

	return angle_b, angle_r

def naka_rushton(m, s, N, u):
	u = u * (u >= 0)
	return (m * u**N)/(s**N + u**N)

def x_1(neu, S1, x_2, l):
	return (1/tau_1)*(-neu + naka_rushton(5, 1.3, 2, S1 + w_11*neu - w_12*x_2 + l))

def x_2(neu, S2, x_1, l):
	return (1/tau_2)*(-neu + naka_rushton(10, 3.6 + 1.4*(l > 0), 2, S2 + w_22*neu - w_21*x_1 + l))

def x_3(neu, x_1, x_2):
	return (1/tau_2)*(-neu + naka_rushton(10, 3.6, 2, w_31*x_1 - w_32*x_2))

def x_4(neu, x_1, x_2):
	return (1/tau_2)*(-neu + naka_rushton(10, 3.6, 2, w_41*x_2 - w_42*x_1))

def x_5(neu, x_3, x_4, x_6, x_7):
	return (1/0.25)*(-neu + naka_rushton(5, 0.004, 2, w_54*x_4 + w_53*x_3 - w_56*x_6 - w_57*x_7  + 0.1*neu))

def x_6(neu, x_1):
	return (1/0.2)*(-neu + naka_rushton(5, 3, 2, x_1 + 0.8*neu))

def x_7(neu, x_2):
	return (1/0.2)*(-neu + naka_rushton(5, 4, 2, x_2 + 0.8*neu))



j = 1
class ObjectDistanceRetrieval(object):



	def __init__(self):

		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/hexapod_master/camera1/image_raw", Image, self.camera_callback)
		self.im_pub = rospy.Publisher("/modifi", Image, queue_size = 1)
		self.action_pub = rospy.Publisher("/ActionSelection", String, queue_size = 1)
		self.setpoint = rospy.Publisher('/setpoint', Num, queue_size = 1)

	def camera_callback(self, data):
		global X_1, X_2, X_3, X_4, X_5, X_6, X_7, t, k, j

		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding = 'bgr8')
		except CvBridgeError as e:
			print(e)

		w_b, h_b, w_r, h_r, location_b, location_r, image_mod = return_signal(cv_image)



		if w_b == 0:
			D_b = 100
		else:
			D_b = 0.5*489/w_b
		if w_r == 0:
			D_r = 100
		else:
			D_r = 0.5*489/w_r

		#angle_blue, angle_red = calc_angle(location_b, D_b, location_r, D_r)
		angle_blue, angle_red = calc_angle(location_b, location_r)
		#angle_blue = angle_blue + 90*(b_i==1 and angle_blue > 0) - 90*(b_i==1 and angle_blue < 0)
		#angle_red = angle_red + 90*(r_i==1 and angle_red > 0) - 90*(r_i==1 and angle_red < 0)
		if len(angle_blue) != 0:

			self.setpoint.publish(np.array(angle_blue))


		print("--------------x-----------------")
		print([angle_blue, angle_red])

		print([location_b, location_r])



		S_1 = (D_r < 4.2)*(m_r*D_r + b_r) + (D_r >= 4.2 and D_r < 8)*(m_i*D_r + b_i+0.02) + (D_r >= 8)*(0.11) #Predator signal
		S_2 = (D_b < 4.2)*(m_b*D_b + b_b) + (D_b >= 4.2 and D_b < 8)*(m_i*D_b + b_i) + (D_b >= 8)*(0.11) #Prey signal

		s1 = S_1 + 0.05 * S_1 * np.random.normal(0,1)*0
		s2 = S_2 + 0.05 * S_2 * np.random.normal(0,1)*0
		L_m = L + 0.05 * L * np.random.normal(0,1)*0

		k1_X_1 = h * x_1(X_1[0], s1*(L == 0), X_2[0], L_m)
		k1_X_2 = h * x_2(X_2[0], s2*(L == 0), X_1[0], L_m)
		k1_X_3 = h * x_3(X_3[0], X_1[0], X_2[0])
		k1_X_4 = h * x_4(X_4[0], X_1[0], X_2[0])
		k1_X_5 = h * x_5(X_5[0], X_3[0], X_4[0], X_6[0], X_7[0])
		k1_X_6 = h * x_6(X_6[0], X_1[0])
		k1_X_7 = h * x_7(X_7[0], X_2[0])


		k2_X_1 = h * x_1(X_1[0] + k1_X_1 * h/2, s1*(L == 0), X_2[0] + k1_X_2 * h/2, L_m)
		k2_X_2 = h * x_2(X_2[0] + k1_X_2 * h/2, s2*(L == 0), X_1[0] + k1_X_1 * h/2, L_m)
		k2_X_3 = h * x_3(X_3[0] + k1_X_3 * h/2, X_1[0] + k1_X_1 * h/2, X_2[0] + k1_X_2 * h/2)
		k2_X_4 = h * x_4(X_4[0] + k1_X_4 * h/2, X_1[0] + k1_X_1 * h/2, X_2[0] + k1_X_2 * h/2)
		k2_X_5 = h * x_5(X_5[0] + k1_X_5 * h/2, X_3[0] + k1_X_3 * h/2, X_4[0] + k1_X_4 * h/2, X_6[0] + k1_X_6 * h/2, X_7[0] + k1_X_7 * h/2)
		k2_X_6 = h * x_6(X_6[0] + k1_X_6 * h/2, X_1[0] + k1_X_1 * h/2)
		k2_X_7 = h * x_7(X_7[0] + k1_X_7 * h/2, X_2[0] + k1_X_2 * h/2)


		k3_X_1 = h * x_1(X_1[0] + k2_X_1 * h/2, s1*(L == 0), X_2[0] + k2_X_2 * h/2, L_m)
		k3_X_2 = h * x_2(X_2[0] + k2_X_2 * h/2, s2*(L == 0), X_1[0] + k2_X_1 * h/2, L_m)
		k3_X_3 = h * x_3(X_3[0] + k2_X_3 * h/2, X_1[0] + k2_X_1 * h/2, X_2[0] + k2_X_2 * h/2)
		k3_X_4 = h * x_4(X_4[0] + k2_X_4 * h/2, X_1[0] + k2_X_1 * h/2, X_2[0] + k2_X_2 * h/2)
		k3_X_5 = h * x_5(X_5[0] + k2_X_5 * h/2, X_3[0] + k2_X_3 * h/2, X_4[0] + k2_X_4 * h/2, X_6[0] + k2_X_6 * h/2, X_7[0] + k2_X_7 * h/2)
		k3_X_6 = h * x_6(X_6[0] + k2_X_6 * h/2, X_1[0] + k2_X_1 * h/2)
		k3_X_7 = h * x_7(X_7[0] + k2_X_7 * h/2, X_2[0] + k2_X_2 * h/2)

		k4_X_1 = h * x_1(X_1[0] + k3_X_1 * h, s1*(L == 0), X_2[0] + k3_X_2 * h, L_m)
		k4_X_2 = h * x_2(X_2[0] + k3_X_2 * h, s2*(L == 0), X_1[0] + k3_X_1 * h, L_m)
		k4_X_3 = h * x_3(X_3[0] + k3_X_3 * h, X_1[0] + k3_X_1 * h, X_2[0] + k3_X_2 * h)
		k4_X_4 = h * x_4(X_4[0] + k3_X_4 * h, X_1[0] + k3_X_1 * h, X_2[0] + k3_X_2 * h)
		k4_X_5 = h * x_5(X_5[0] + k3_X_5 * h, X_3[0] + k3_X_3 * h, X_4[0] + k3_X_4 * h, X_6[0] + k3_X_6 * h, X_7[0] + k3_X_7 * h)
		k4_X_6 = h * x_6(X_6[0] + k3_X_6 * h, X_1[0] + k3_X_1 * h)
		k4_X_7 = h * x_7(X_7[0] + k3_X_7 * h, X_2[0] + k3_X_2 * h)



		X_1[1] = X_1[0] + (1/6)*(k1_X_1 + 2*k2_X_1 + 2*k3_X_1 + k4_X_1)
		X_2[1] = X_2[0] + (1/6)*(k1_X_2 + 2*k2_X_2 + 2*k3_X_2 + k4_X_2)
		X_3[1] = X_3[0] + (1/6)*(k1_X_3 + 2*k2_X_3 + 2*k3_X_3 + k4_X_3)
		X_4[1] = X_4[0] + (1/6)*(k1_X_4 + 2*k2_X_4 + 2*k3_X_4 + k4_X_4)
		X_5[1] = X_5[0] + (1/6)*(k1_X_5 + 2*k2_X_5 + 2*k3_X_5 + k4_X_5)
		X_6[1] = X_6[0] + (1/6)*(k1_X_6 + 2*k2_X_6 + 2*k3_X_6 + k4_X_6)
		X_7[1] = X_7[0] + (1/6)*(k1_X_7 + 2*k2_X_7 + 2*k3_X_7 + k4_X_7)


		X_1[0] = X_1[1]
		X_2[0] = X_2[1]
		X_3[0] = X_3[1]
		X_4[0] = X_4[1]
		X_5[0] = X_5[1]
		X_6[0] = X_6[1]
		X_7[0] = X_7[1]




		F = np.array([X_5[1], X_6[1], X_7[1]])
		f = np.argmax(F)
		if f == 0:
			Out = "explore"
		if f == 1:
			Out = "escape"
		if f == 2:
			Out = "attack"

		self.action_pub.publish(Out)
		# self.im_pub.publish(self.bridge_object.cv2_to_imgmsg(image_mod,  encoding="passthrough"))
		print(Out)
		print([S_2, S_1])


		k += 1
		t += 1





def main(args):
	line_follower_object = ObjectDistanceRetrieval()
	rospy.init_node('distance_retrieval_node', anonymous = True)
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
