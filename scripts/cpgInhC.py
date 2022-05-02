#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from matplotlib import pyplot as plt
from hexapodo.msg import Num

pub=[]
data=0.0
h = 0.1
stime = 1000000
time = np.linspace(0, stime, int(stime/h))

tau=[0.4, 50] #[0tA,1tB,2tc]
K1 = 150
K2 = 150
R = 1
L = 1
contador = 0
cont=0

def hexapod_direct(data):
	global L, R
	direct = data.num
	L = direct[0]
	R = direct[1]

def nE(E1, E2, A, Tau, K):
	return (1/Tau)*(-E1+NR2(E2, A, K))

def nAp(A, E, Tau):
	return (1/Tau)*(-A+1.5*E)

def nP1(E1, P1, P2):
	return (1/0.05)*(-P1+ NR(E1,2.5*P2))

def nP2(P1, P2):
	return (1/50)*(-P2+ NR(P1 + 0.4*P2, 50))

def Out(Pout, P1):
	return (1/1)*(-Pout+ NR(P1 -20, 15))

def NR(x,S):
	return 100 * ((x * (x > 0)) ** 2)/ ((S ** 2) + ((x * (x > 0)) ** 2))

def NR2(E,A,K):
	return 100 * (((K-3.2*E) * (E < K/3.2)) ** 2)/ (((120+A) ** 2) + (((K-3.2*E) * (E < K/3.2)) ** 2))

def RK4(stime, h):
	n = int(stime/h)
        z1 = np.zeros((18,n))

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

	rospy.init_node('hexapod_talker', anonymous=True)
	rate = rospy.Rate(1000) # 10hz
	H=0
	rospy.Subscriber("/hexapod_direct", Num, hexapod_direct)

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

		P1I[1] = P1I[0] + (1/6)*(k1_P1I + 2 * k2_P1I + 2 * k3_P1I + k4_P1I)
		P2I[1] = P2I[0] + (1/6)*(k1_P2I + 2 * k2_P2I + 2 * k3_P2I + k4_P2I)
		PoutI[1]=PoutI[0] + (1/6)*(k1_OutI + 2 * k2_OutI + 2 * k3_OutI + k4_OutI)

		P1D[1] = P1D[0] + (1/6)*(k1_P1D + 2 * k2_P1D + 2 * k3_P1D + k4_P1D)
		P2D[1] = P2D[0] + (1/6)*(k1_P2D + 2 * k2_P2D + 2 * k3_P2D + k4_P2D)
		PoutD[1]=PoutD[0] + (1/6)*(k1_OutD + 2 * k2_OutD + 2 * k3_OutD + k4_OutD)

		T1[1] = (E1[1]/70)-0.4
		ToutI[1] = PoutI[1]/100
		T2[1] = (E2[1]/70)-0.4
		ToutD[1] = PoutD[1]/100

		E1[0] = E1[1]
		E2[0] = E2[1]
		A1[0] = A1[1]
		A2[0] = A2[1]

		P1I[0]  = P1I[1]
		P2I[0]  = P2I[1]
		PoutI[0]= PoutI[1]

		global contador
		global cont
		contador += 1
		Tp[1]=T1[1]
		Tp2[1]=ToutI[1]
		Tp3[1]=T2[1]
		Tp4[1]=ToutD[1]

		if contador <50:
			Tp[1]=0
			Tp3[1]=0


		z=0
		if R==0 and L==0:
			for i in range(0,6):
				for j in range(0,3):

        	                        z1[0,1]=Tp[1]
        	                        z1[1,1]=Tp2[1]
        	                        z1[2,1]=0

        	                        z1[3,1]=Tp3[1]
        	                        z1[4,1]=Tp4[1]
        	                        z1[5,1]=0

        	                        z1[6,1]=Tp[1]
        	                        z1[7,1]=Tp2[1]
        	                        z1[8,1]=0

        	                        z1[9,1]=-Tp3[1]
        	                        z1[10,1]=Tp4[1]
        	                        z1[11,1]=0

        	                        z1[12,1]=-Tp[1]
        	                        z1[13,1]=Tp2[1]
        	                        z1[14,1]=0

        	                        z1[15,1]=-Tp3[1]
        	                        z1[16,1]=Tp4[1]
        	                        z1[17,1]=0

					if cont<500:
						print(cont)
						pub[0].publish(0)
						pub[1].publish(0)
						pub[2].publish(0)
						pub[3].publish(0)
						pub[4].publish(0)
						pub[5].publish(0)
						pub[6].publish(0)
						pub[7].publish(0)
						pub[8].publish(0)
						pub[9].publish(0)
						pub[10].publish(0)
						pub[11].publish(0)
						pub[12].publish(0)
						pub[13].publish(0)
						pub[14].publish(0)
						pub[15].publish(0)
						pub[16].publish(0)
						pub[17].publish(0)
						cont += 1
					else:
        	                        	pub[z].publish(z1[z,1])
					z+=1
			rate.sleep()

		if R==1 and L==1:
			for i in range(0,6):
				for j in range(0,3):

        	                        z1[0,1]=0
        	                        z1[1,1]=0
        	                        z1[2,1]=0

        	                        z1[3,1]=0
        	                        z1[4,1]=0
        	                        z1[5,1]=0

        	                        z1[6,1]=0
        	                        z1[7,1]=0
        	                        z1[8,1]=0

        	                        z1[9,1]=0
        	                        z1[10,1]=0
        	                        z1[11,1]=0

        	                        z1[12,1]=0
        	                        z1[13,1]=0
        	                        z1[14,1]=0

        	                        z1[15,1]=0
        	                        z1[16,1]=0
        	                        z1[17,1]=0

					if cont<500:
						print(cont)
						pub[0].publish(0)
						pub[1].publish(0)
						pub[2].publish(0)
						pub[3].publish(0)
						pub[4].publish(0)
						pub[5].publish(0)
						pub[6].publish(0)
						pub[7].publish(0)
						pub[8].publish(0)
						pub[9].publish(0)
						pub[10].publish(0)
						pub[11].publish(0)
						pub[12].publish(0)
						pub[13].publish(0)
						pub[14].publish(0)
						pub[15].publish(0)
						pub[16].publish(0)
						pub[17].publish(0)
						cont += 1
					else:
        	                        	pub[z].publish(z1[z,1])
					z+=1
			rate.sleep()

		if R==1 and L==0:
			for i in range(0,6):
				for j in range(0,3):

        	                        z1[0,1]=-T1[1]
        	                        z1[1,1]=ToutI[1]
        	                        z1[2,1]=0

        	                        z1[3,1]=-T2[1]
        	                        z1[4,1]=ToutD[1]
        	                        z1[5,1]=0

        	                        z1[6,1]=-T1[1]
        	                        z1[7,1]=ToutI[1]
        	                        z1[8,1]=0

        	                        z1[9,1]=-T2[1]
        	                        z1[10,1]=ToutD[1]
        	                        z1[11,1]=0

        	                        z1[12,1]=-T1[1]
        	                        z1[13,1]=ToutI[1]
        	                        z1[14,1]=0

        	                        z1[15,1]=-T2[1]
        	                        z1[16,1]=ToutD[1]
        	                        z1[17,1]=0


        	                        pub[z].publish(z1[z,1])
					z+=1
			rate.sleep()

		if R==0 and L==1:
			for i in range(0,6):
				for j in range(0,3):

        	                        z1[0,1]=T1[1]
        	                        z1[1,1]=ToutI[1]
        	                        z1[2,1]=0

        	                        z1[3,1]=T2[1]
        	                        z1[4,1]=ToutD[1]
        	                        z1[5,1]=0

        	                        z1[6,1]=T1[1]
        	                        z1[7,1]=ToutI[1]
        	                        z1[8,1]=0

        	                        z1[9,1]=T2[1]
        	                        z1[10,1]=ToutD[1]
        	                        z1[11,1]=0

        	                        z1[12,1]=T1[1]
        	                        z1[13,1]=ToutI[1]
        	                        z1[14,1]=0

        	                        z1[15,1]=T2[1]
        	                        z1[16,1]=ToutD[1]
        	                        z1[17,1]=0


        	                        pub[z].publish(z1[z,1])
					z+=1
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
