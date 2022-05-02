#!/usr/bin/env python3
# license removed for brevity
from __future__ import division
from __future__ import print_function
import rospy
from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Twist
import math
import numpy as np
from matplotlib import pyplot as plt
from hexapodo.msg import Num

pub=[]
data=0.0
h = 0.1
stime = 1000000
time = np.linspace(0, stime, int(stime/h))

tau=[0.4, 50]
K1=150
K2=150
R=1
L=1
contador = 0
cont=0
cpl=0
cpr=0
x0l=0
x3l=0
x0r=0
x3r=0
xt=0
insta_distance_reset = 0
rotation_direction_scale = 0
rotation_direction = 0
left_scaling = 0
right_scaling = 0
def hexapod_direct(data):
	global L, R
	direct = data.num
	L = direct[0]
	R = direct[1]


def Rotation_direction_scale(data):
	global rotation_direction_scale
	rotation_direction_scale = data.data

def Rotation_direction(data):
	global rotation_direction
	rotation_direction = data.data

def L_R_Scaling(data):
	global left_scaling, right_scaling
	left_scaling = data.num[0]
	right_scaling = data.num[1]

def Insta_distance_reset(data):
	global insta_distance_reset
	insta_distance_reset = data.data

def nE(E1, E2, A, Tau, K):
	return (1/Tau)*(-E1+NR2(E2, A, K))

def nAp(A, E, Tau):
	return (1/Tau)*(-A+1.5*E)

def nP1(E1, P1, P2):
	return (1/0.05)*(-P1+ NR(E1,2.5*P2))

def nP2(P1, P2):
	return (1/50)*(-P2+ NR(P1 + 0.4 *P2, 50))

def Out(Pout, P1):
	return (1/1)*(-Pout+ NR(P1 -20, 15))

def NR(x,S):
	return 100 * ((x * (x > 0)) ** 2)/ ((S ** 2) + ((x * (x > 0)) ** 2))

def NR2(E,A,K):
	return 100 * (((K-3.2*E) * (E < K/3.2)) ** 2)/ (((120+A) ** 2) + (((K-3.2*E) * (E < K/3.2)) ** 2))

def RK4(stime, h):
	n = int(stime/h)
	z11 = np.zeros((18,n))

	E1 = np.zeros((2))
	T1 = np.zeros((2))
	E1[0]=1
	E2 = np.zeros((2))
	E2[0]=0.000000001
	T2 = np.zeros((2))

	A1 = np.zeros((2))
	A2 = np.zeros((2))

	P1I = np.zeros((2))
	P2I= np.zeros((2))
	PoutI = np.zeros((2))
	ToutI = np.zeros((2))

	P1D = np.zeros((2))
	P2D= np.zeros((2))
	PoutD = np.zeros((2))
	ToutD = np.zeros((2))

	Tp=np.zeros((2))
	Tp2=np.zeros((2))
	Tp3=np.zeros((2))
	Tp4=np.zeros((2))
	Tp5=np.zeros((2))
	Tp6=np.zeros((2))

	global contador, cont, cpl, x0l, x1, x2, x3l, xt, cpr, x0r, x3r, y0l, y3l, y0r, y3r, yt, insta_distance_reset, rotation_direction, L, R

	rospy.init_node('hexapod_talker', anonymous=True)
	rospy.Subscriber("/hexapod_direct", Num, hexapod_direct)
	rospy.Subscriber("/insta_distance_reset", Int64, Insta_distance_reset)
	rospy.Subscriber("/rotation_direction_scale", Float64, Rotation_direction_scale)
	rospy.Subscriber("/rotation_direction", Int64, Rotation_direction)
	rospy.Subscriber("/L_R_scaling", Num, L_R_Scaling)
	insta_distance = rospy.Publisher("/insta_distance", Float64, queue_size = 1)
	Twister = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	twist = Twist()
	rate = rospy.Rate(2000) # 10hz
	H=0

	for k in range(1,7):
		for l in range(1,4):
			pub.append(rospy.Publisher('/hexapodo/jp'+str(k)+str(l)+'_position_controller/command', Float64, queue_size=10))

	while not rospy.is_shutdown():


		

		

		k1_E1 = h * nE(E1[0], E2[0], A1[0], tau[0], K1)
		k1_E2 = h * nE(E2[0], E1[0], A2[0], tau[0], K2)
		k1_A1 = h * nAp(A1[0], E1[0], tau[1])
		k1_A2 = h * nAp(A2[0], E2[0], tau[1])

		k1_P1I = h * nP1(E1[0], P1I[0], P2I[0])
		k1_P2I = h * nP2(P1I[0], P2I[0])
		k1_OutI = h * Out(PoutI[0], P1I[0])

		k1_P1D = h * nP1(E2[0], P1D[0], P2D[0])
		k1_P2D = h * nP2(P1D[0], P2D[0])
		k1_OutD = h * Out(PoutD[0], P1D[0])

		k2_E1 = h * nE(E1[0] + k1_E1 * h/2, E2[0] + k1_E2 * h/2, A1[0] + k1_A1 * h/2, tau[0], K1)
		k2_E2 = h * nE(E2[0] + k1_E2 * h/2, E1[0] + k1_E1 * h/2, A2[0] + k1_A2 * h/2, tau[0], K2)
		k2_A1 = h * nAp(A1[0] + k1_A1 * h/2, E1[0] + k1_E1 * h/2, tau[1])
		k2_A2 = h * nAp(A2[0] + k1_A2 * h/2, E2[0] + k1_E2 * h/2, tau[1])

		k2_P1I = h * nP1(E1[0] + k1_E1 * h/2, P1I[0] + k1_P1I * h/2, P2I[0] + k1_P2I * h/2)
		k2_P2I = h * nP2(P1I[0] + k1_P1I * h/2, P2I[0] + k1_P2I * h/2)
		k2_OutI = h * Out(PoutI[0] + k1_OutI * h/2, P1I[0] + k1_P1I * h/2)

		k2_P1D = h * nP1(E2[0] + k1_E2 * h/2, P1D[0] + k1_P1D * h/2, P2D[0] + k1_P2D * h/2)
		k2_P2D = h * nP2(P1D[0] + k1_P1D * h/2, P2D[0] + k1_P2D * h/2)
		k2_OutD = h * Out(PoutD[0] + k1_OutD * h/2, P1D[0] + k1_P1D * h/2)

		k3_E1 = h * nE(E1[0] + k2_E1 * h/2, E2[0] + k2_E2 * h/2, A1[0] + k1_A1 * h/2, tau[0], K1)
		k3_E2 = h * nE(E2[0] + k2_E2 * h/2, E1[0] + k2_E1 * h/2, A2[0] + k1_A2 * h/2, tau[0], K2)
		k3_A1 = h * nAp(A1[0] + k1_A1 * h/2, E1[0] + k2_E1 * h/2, tau[1])
		k3_A2 = h * nAp(A2[0] + k1_A2 * h/2, E2[0] + k2_E2 * h/2, tau[1])

		k3_P1I = h * nP1(E1[0] + k2_E1 * h/2, P1I[0] + k2_P1I * h/2, P2I[0] + k2_P2I * h/2)
		k3_P2I = h * nP2(P1I[0] + k2_P1I * h/2, P2I[0] + k2_P2I * h/2)
		k3_OutI = h * Out(PoutI[0] + k2_OutI * h/2, P1I[0] + k2_P1I * h/2)

		k3_P1D = h * nP1(E2[0] + k2_E2 * h/2, P1D[0] + k2_P1D * h/2, P2D[0] + k2_P2D * h/2)
		k3_P2D = h * nP2(P1D[0] + k2_P1D * h/2, P2D[0] + k2_P2D * h/2)
		k3_OutD = h * Out(PoutD[0] + k2_OutD * h/2, P1D[0] + k2_P1D * h/2)

		k4_E1 = h * nE(E1[0] + k3_E1 * h, E2[0] + k3_E2 * h, A1[0] + k1_A1 * h, tau[0], K1)
		k4_E2 = h * nE(E2[0] + k3_E2 * h, E1[0] + k3_E1 * h, A2[0] + k1_A2 * h, tau[0], K2)
		k4_A1 = h * nAp(A1[0] + k1_A1 * h, E1[0] + k1_E1 * h, tau[1])
		k4_A2 = h * nAp(A2[0] + k1_A2 * h, E2[0] + k1_E2 * h, tau[1])

		k4_P1I = h * nP1(E1[0] + k3_E1 * h, P1I[0] + k3_P1I * h, P2I[0] + k3_P2I * h)
		k4_P2I = h * nP2(P1I[0] + k3_P1I * h, P2I[0] + k3_P2I * h)
		k4_OutI = h * Out(PoutI[0] + k3_OutI * h, P1I[0] + k3_P1I * h)

		k4_P1D = h * nP1(E2[0] + k3_E2 * h, P1D[0] + k3_P1D * h, P2D[0] + k3_P2D * h)
		k4_P2D = h * nP2(P1D[0] + k3_P1D * h, P2D[0] + k3_P2D * h)
		k4_OutD = h * Out(PoutD[0] + k3_OutD * h, P1D[0] + k3_P1D * h)

		E1[1] = E1[0] + (1/6)*(k1_E1 + 2 * k2_E1 + 2 * k3_E1 + k4_E1)
		E2[1] = E2[0] + (1/6)*(k1_E2 + 2 * k2_E2 + 2 * k3_E2 + k4_E2)
		A1[1] = A1[0] + (1/6)*(k1_A1 + 2 * k2_A1 + 2 * k3_A1 + k4_A1)
		A2[1] = A2[0] + (1/6)*(k1_A2 + 2 * k2_A2 + 2 * k3_A2 + k4_A2)

		P1I[1] = P1I[0]   + (1/6)*(k1_P1I + 2 * k2_P1I + 2 * k3_P1I + k4_P1I)
		P2I[1] = P2I[0]   + (1/6)*(k1_P2I + 2 * k2_P2I + 2 * k3_P2I + k4_P2I)
		PoutI[1]=PoutI[0] + (1/6)*(k1_OutI + 2 * k2_OutI + 2 * k3_OutI + k4_OutI)

		P1D[1] = P1D[0]   + (1/6)*(k1_P1D + 2 * k2_P1D + 2 * k3_P1D + k4_P1D)
		P2D[1] = P2D[0]   + (1/6)*(k1_P2D + 2 * k2_P2D + 2 * k3_P2D + k4_P2D)
		PoutD[1]=PoutD[0] + (1/6)*(k1_OutD + 2 * k2_OutD + 2 * k3_OutD + k4_OutD)

		T1[1] = (E1[1]/70)-0.4
		ToutI[1] = PoutI[1]/100
		T2[1] = (E2[1]/70)-0.4
		ToutD[1] = PoutD[1]/100

		E1[0] = E1[1]
		E2[0] = E2[1]
		A1[0] = A1[1]
		A2[0] = A2[1]

		P1I[0] = P1I[1]
		P2I[0] = P2I[1]
		PoutI[0]=PoutI[1]

		P1D[0] = P1D[1]
		P2D[0] = P2D[1]
		PoutD[0]=PoutD[1]

		contador += 1
		Tp[1]=T1[1]
		Tp2[1]=ToutI[1]
		Tp3[1]=0
		Tp4[1]=T2[1]
		Tp5[1]=ToutD[1]
		Tp6[1]=0

		rot_scaling = (0.9/100)*np.abs(rotation_direction_scale) + 0.1

		# if contador <80:



		if R==0 and L==0:

			AH1=Tp[1] +1.5708
			AC1=Tp2[1]
			AM1=Tp3[1]-1.5708

			AH2=Tp4[1] +1.5708
			AC2=Tp5[1]
			AM2=Tp6[1]-1.5708

			x1=-(0.17615998878)*(math.sin(AC1)*math.sin(AM1)*math.cos(AH1))+(0.17615998878)*(math.cos(AH1)*math.cos(AC1)*math.cos(AM1))+(2/25)*(math.cos(AH1)*math.cos(AC1))+(3/50)*(math.cos(AH1))
			y1=-(0.17615998878)*(math.sin(AH1)*math.sin(AC1)*math.sin(AM1))+(0.17615998878)*(math.sin(AH1)*math.cos(AC1)*math.cos(AM1))+(2/25)*(math.sin(AH1)*math.cos(AC1))+(3/50)*(math.sin(AH1))
			z1= (0.17615998878)*(math.sin(AC1)*math.cos(AM1))+(2/25)*(math.sin(AC1))+(0.17615998878)*(math.sin(AM1)*math.cos(AC1))



			x2=-(0.17615998878)*(math.sin(AC2)*math.sin(AM2)*math.cos(AH2))+(0.17615998878)*(math.cos(AH2)*math.cos(AC2)*math.cos(AM2))+(2/25)*(math.cos(AH2)*math.cos(AC2))+(3/50)*(math.cos(AH2))
			y2=-(0.17615998878)*(math.sin(AH2)*math.sin(AC2)*math.sin(AM2))+(0.17615998878)*(math.sin(AH2)*math.cos(AC2)*math.cos(AM2))+(2/25)*(math.sin(AH2)*math.cos(AC2))+(3/50)*(math.sin(AH2))
			z2= (0.17615998878)*(math.sin(AC2)*math.cos(AM2))+(2/25)*(math.sin(AC2))+(0.17615998878)*(math.sin(AM2)*math.cos(AC2))
			if z1<-0.17 and cpl==0 :
				xt+=(-x0l+x3l)+(-x0l+x3l)*0.11809
				x0l=x1
				cpl=1

				# print("xt: "+str(xt))
			if z1>-0.17 and cpl==1 :
				x3l=x1
				cpl=0

			if z2<-0.17 and cpr==0 :
				xt+=(-x0r+x3r)+(-x0r+x3r)*0.11809
				x0r=x2
				cpr=1
				# print("xt: "+str(xt))
			if z2>-0.17 and cpr==1 :
				x3r=x2
				cpr=0
			if insta_distance_reset == 1:
				xt = 0
				insta_distance_reset = 0

			insta_distance.publish(np.abs(xt))
			#xt = 3
			#insta_distance.publish(xt)
			print("insta_distance: "+str(xt))
			print([L, R])


			# z11[0,1]=Tp[1]
			# z11[1,1]=Tp2[1]
			# z11[2,1]=Tp3[1]
			# z11[3,1]=Tp4[1]
			# z11[4,1]=Tp5[1]
			# z11[5,1]=Tp6[1]
			# z11[6,1]=Tp[1]
			# z11[7,1]=Tp2[1]
			# z11[8,1]=Tp3[1]
			# z11[9,1]=-Tp4[1]
			# z11[10,1]=Tp5[1]
			# z11[11,1]=Tp6[1]
			# z11[12,1]=-Tp[1]
			# z11[13,1]=Tp2[1]
			# z11[14,1]=Tp3[1]
			# z11[15,1]=-Tp4[1]
			# z11[16,1]=Tp5[1]
			# z11[17,1]=Tp6[1]

			z11[0,1]=right_scaling*Tp[1]
			z11[1,1]=Tp2[1]
			z11[2,1]=Tp3[1]
			z11[3,1]=right_scaling*Tp4[1]
			z11[4,1]=Tp5[1]
			z11[5,1]=Tp6[1]
			z11[6,1]=right_scaling*Tp[1]
			z11[7,1]=Tp2[1]
			z11[8,1]=Tp3[1]
			z11[9,1]=-left_scaling*Tp4[1]
			z11[10,1]=Tp5[1]
			z11[11,1]=Tp6[1]
			z11[12,1]=-left_scaling*Tp[1]
			z11[13,1]=Tp2[1]
			z11[14,1]=Tp3[1]
			z11[15,1]=-left_scaling*Tp4[1]
			z11[16,1]=Tp5[1]
			z11[17,1]=Tp6[1]


			for z in range(0,18):
				pub[z].publish(z11[z,1])

			twist.linear.x = 0.05; twist.linear.y = 0; twist.linear.z = 0;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


			


		if R==1 and L==1:
			for z in range(0,18):

				z11[0,1]=0.0
				z11[1,1]=0.0
				z11[2,1]=0.0
				z11[3,1]=0.0
				z11[4,1]=0.0
				z11[5,1]=0.0
				z11[6,1]=0.0
				z11[7,1]=0.0
				z11[8,1]=0.0
				z11[9,1]=0.0
				z11[10,1]=0.0
				z11[11,1]=0.0
				z11[12,1]=0.0
				z11[13,1]=0.0
				z11[14,1]=0.0
				z11[15,1]=0.0
				z11[16,1]=0.0
				z11[17,1]=0.0
				pub[z].publish(z11[z,1])

				twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


				Twister.publish(twist)

		if R==1 and L==0:
			for z in range(0,18):

				z11[0,1]=-rot_scaling*T1[1]
				z11[1,1]=ToutI[1]
				z11[2,1]=0
				z11[3,1]=-rot_scaling*T2[1]
				z11[4,1]=ToutD[1]
				z11[5,1]=0
				z11[6,1]=-rot_scaling*T1[1]
				z11[7,1]=ToutI[1]
				z11[8,1]=0
				z11[9,1]=-rot_scaling*T2[1]
				z11[10,1]=ToutD[1]
				z11[11,1]=0
				z11[12,1]=-rot_scaling*T1[1]
				z11[13,1]=ToutI[1]
				z11[14,1]=0
				z11[15,1]=-rot_scaling*T2[1]
				z11[16,1]=ToutD[1]
				z11[17,1]=0
				pub[z].publish(z11[z,1])

				twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.2


				

		if R==0 and L==1:
			for z in range(0,18):

				z11[0,1]=rot_scaling*T1[1]
				z11[1,1]=ToutI[1]
				z11[2,1]=0
				z11[3,1]=rot_scaling*T2[1]
				z11[4,1]=ToutD[1]
				z11[5,1]=0
				z11[6,1]=rot_scaling*T1[1]
				z11[7,1]=ToutI[1]
				z11[8,1]=0
				z11[9,1]=rot_scaling*T2[1]
				z11[10,1]=ToutD[1]
				z11[11,1]=0
				z11[12,1]=rot_scaling*T1[1]
				z11[13,1]=ToutI[1]
				z11[14,1]=0
				z11[15,1]=rot_scaling*T2[1]
				z11[16,1]=ToutD[1]
				z11[17,1]=0
				pub[z].publish(z11[z,1])

				twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.2

		Twister.publish(twist)

		print(("L: {left_var} - - R: {right_var}").format(left_var = L, right_var = R))
		rate.sleep()


def talker():
	t=0
	jp=([11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63])
	RK4(stime, h)


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
