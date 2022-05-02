#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from matplotlib import pyplot as plt

w=[1.6,1.5,1,1,1,1,1,1,1,1] #[0wAA, 1wAB, 2wBA, 3wAc,4wBc,5wca,6waa,7wab,8wba,9wLA]
tau=[5,150,20] #[0tA,1tB,2tc]
taua=[80,5,5]
K1=150
K2=150
pub=[]
data=0.0
h = 1
stime = 10000
time = np.linspace(0, stime, int(stime/h))

def nA(A, C, tau, WL):
	return (1/tau)*(-A + NR(WL*C, 30))

def nC(C, E1, E2, tau, WA):
	return (1/tau)*(-C + NR(WA*E1, 30) - NR(WA*E2, 30))

def nE(E1, E2, A, Tau, K):
	return (1/Tau)*(-E1+NR2(E2, A, K))

def nAp(A, E, Tau):
	return (1/Tau)*(-A+1.5*E)

def NR(x,S):
	return 100 * ((x * (x > 0)) ** 2)/ ((S ** 2) + ((x * (x > 0)) ** 2))

def NR2(E,A,K):
	return 100 * (((K-3.2*E) * (E < K/3.2)) ** 2)/ (((120+A) ** 2) + (((K-3.2*E) * (E < K/3.2)) ** 2))

def RK4(time, h):
	n = int(time/h)
	a = np.zeros((18,n))
	g = np.zeros((18,n))
	c = np.zeros((6,n))
	E1 = np.zeros((n))
	E1[0]=10
	E2 = np.zeros((n))
	A1 = np.zeros((n))
	A2 = np.zeros((n))

	rospy.init_node('hexapod_talker', anonymous=True)
	rate = rospy.Rate(1000000000000000000) # 10hz

	for k in range(1,7):
		for l in range(1,4):
			pub.append(rospy.Publisher('/hexapodo/jp'+str(k)+str(l)+'_position_controller/command', Float64, queue_size=10))

	for t in range(1, n):

		k1_E1 = h * nE(E1[t - 1], E2[t - 1], A1[t - 1], tau[0], K1)
		k1_E2 = h * nE(E2[t - 1], E1[t - 1], A2[t - 1], tau[0], K2)
		k1_A1 = h * nAp(A1[t - 1], E1[t - 1], tau[1])
		k1_A2 = h * nAp(A2[t - 1], E2[t - 1], tau[1])

		k2_E1 = h * nE(E1[t - 1] + k1_E1 * h/2, E2[t - 1] + k1_E2 * h/2, A1[t - 1] + k1_A1 * h/2, tau[0], K1)
		k2_E2 = h * nE(E2[t - 1] + k1_E2 * h/2, E1[t - 1] + k1_E1 * h/2, A2[t - 1] + k1_A2 * h/2, tau[0], K2)
		k2_A1 = h * nAp(A1[t - 1] + k1_A1 * h/2, E1[t - 1] + k1_E1 * h/2, tau[1])
		k2_A2 = h * nAp(A2[t - 1] + k1_A2 * h/2, E2[t - 1] + k1_E2 * h/2, tau[1])

		k3_E1 = h * nE(E1[t - 1] + k2_E1 * h/2, E2[t - 1] + k2_E2 * h/2, A1[t - 1] + k1_A1 * h/2, tau[0], K1)
		k3_E2 = h * nE(E2[t - 1] + k2_E2 * h/2, E1[t - 1] + k2_E1 * h/2, A2[t - 1] + k1_A2 * h/2, tau[0], K2)
		k3_A1 = h * nAp(A1[t - 1] + k1_A1 * h/2, E1[t - 1] + k2_E1 * h/2, tau[1])
		k3_A2 = h * nAp(A2[t - 1] + k1_A2 * h/2, E2[t - 1] + k2_E2 * h/2, tau[1])

		k4_E1 = h * nE(E1[t - 1] + k3_E1 * h, E2[t - 1] + k3_E2 * h, A1[t - 1] + k1_A1 * h, tau[0], K1)
		k4_E2 = h * nE(E2[t - 1] + k3_E2 * h, E1[t - 1] + k3_E1 * h, A2[t - 1] + k1_A2 * h, tau[0], K2)
		k4_A1 = h * nAp(A1[t - 1] + k1_A1 * h, E1[t - 1] + k1_E1 * h, tau[1])
		k4_A2 = h * nAp(A2[t - 1] + k1_A2 * h, E2[t - 1] + k1_E2 * h, tau[1])

		E1[t] = E1[t - 1] + (1/6)*(k1_E1 + 2 * k2_E1 + 2 * k3_E1 + k4_E1)
		E2[t] = E2[t - 1] + (1/6)*(k1_E2 + 2 * k2_E2 + 2 * k3_E2 + k4_E2)
		A1[t] = A1[t - 1] + (1/6)*(k1_A1 + 2 * k2_A1 + 2 * k3_A1 + k4_A1)
		A2[t] = A2[t - 1] + (1/6)*(k1_A2 + 2 * k2_A2 + 2 * k3_A2 + k4_A2)

		z=0
		for i in range(0,6):
			for j in range(0,3):
				k1_A = h * nA(a[z,t - 1], c[i,t-1], taua[j], w[5])
				if i == 0 or i == 2 or i == 4:
					k1_C = h * nC(c[i,t - 1], E1[t - 1], E2[t - 1], tau[2], w[3])
				if i == 1 or i == 3 or i == 5:
					k1_C = h * nC(c[i,t - 1], E2[t - 1], E1[t - 1], tau[2], w[4])

				k2_A = h * nA(a[z,t - 1] + k1_A * h/2, c[i,t-1] + k1_C * h/2, taua[j], w[5])
				if i == 0 or i == 2 or i == 4:
					k2_C = h * nC(c[i,t - 1] + k1_C * h/2, E1[t - 1] + k1_E1 * h/2, E2[t - 1] + k1_E2 * h/2, tau[2], w[3])
				if i == 1 or i == 3 or i == 5:
					k2_C = h * nC(c[i,t - 1] + k1_C * h/2, E2[t - 1] + k1_E2 * h/2, E1[t - 1] + k1_E1 * h/2, tau[2], w[4])

				k3_A = h * nA(a[z,t - 1] + k2_A * h/2, c[i,t-1] + k2_C * h/2, taua[j], w[5])
				if i == 0 or i == 2 or i == 4:
					k3_C = h * nC(c[i,t - 1] + k2_C * h/2, E1[t - 1] + k2_E1 * h/2, E2[t - 1] + k2_E2 * h/2, tau[2], w[3])
				if i == 1 or i == 3 or i == 5:
					k3_C = h * nC(c[i,t - 1] + k2_C * h/2, E2[t - 1] + k2_E2 * h/2, E1[t - 1] + k2_E1 * h/2, tau[2], w[4])

				k4_A = h * nA(a[z,t - 1] + k3_A * h, c[i,t-1] + k3_C * h/2, taua[j], w[5])
				if i == 0 or i == 2 or i == 4:
					k4_C = h * nC(c[i,t - 1] + k3_C * h, E1[t - 1] + k3_E1 * h, E2[t - 1] + k2_E2 * h/2, tau[2], w[3])
				if i == 1 or i == 3 or i == 5:
					k4_C = h * nC(c[i,t - 1] + k3_C * h, E2[t - 1] + k3_E2 * h, E1[t - 1] + k2_E1 * h/2, tau[2], w[4])

				a[z,t] = a[z,t - 1] + (1/6)*(k1_A + 2 * k2_A + 2 * k3_A + k4_A)
				c[i,t] = c[i,t - 1] + (1/6)*(k1_C + 2 * k2_C + 2 * k3_C + k4_C)

				if z==0 or z==3 or z==6 or z==9 or z==12 or z==15:
					g[z,t]=(a[z,t]*1.1/100)-(0.5236)
				if z==1 or z==4 or z==7 or z==10 or z==13 or z==16:
					g[z,t]=(a[z,t]*0.7/100)
				if z==2 or z==5 or z==8 or z==11 or z==14 or z==17:
					g[z,t]=(a[z,t]*-0.5)/100

				pub[z].publish(g[z,t-1])
				rate.sleep()

				z+=1



def talker():
	t=0
	jp=([11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63])
	RK4(stime, h)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

