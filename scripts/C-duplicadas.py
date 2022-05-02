import tkinter
import numpy as np
import math
from matplotlib import pyplot as plt

w=[1.6,1.5,1,1,1,1,1,1,1,1] #[0wAA, 1wAB, 2wBA, 3wAc,4wBc,5wca,6waa,7wab,8wba,9wLA]
vel=1
tau=[vel*1,vel*30,vel*4] #[0tA,1tB,2tc]
taua=[vel*1,vel*1,vel*1]
K1=150
K2=150
asdf=1
asdf1=1

Sigma1=30
Tau1=4
Sigma2=15
Tau2=10

def nC(C1, C0, C2, E1, tau, WA, WI, Sigma):
	return (1/tau)*(-C1 + NR(WA*E1, Sigma)-NR(WI*C0, Sigma)-NR(WI*C2, Sigma))

def nE(E1, E2, A, Tau, K):
	return (1/Tau)*(-E1+NR2(E2, A, K))

def nAp(A, E, Tau):
	return (1/Tau)*(-A + 1.5*E)

def NR(x,S):
	return 100 * ((x * (x > 0)) ** 2)/ ((S ** 2) + ((x * (x > 0)) ** 2))

def NR2(E,A,K):
	return 100 * (((K-3.2*E) * (E < K/3.2)) ** 2)/ (((120+A) ** 2) + (((K-3.2*E) * (E < K/3.2)) ** 2))

def RK4(stime, h):
	n = int(stime/h)
	g = np.zeros((18,n))
	c1 = np.zeros((6,n))
	c2 = np.zeros((6,n))

	E1 = np.zeros((n))
	E1[0]=10
	E2 = np.zeros((n))
	A1 = np.zeros((n))
	A2 = np.zeros((n))

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

				if i == 0 or i == 2:
					k1_C1 = h * nC(c1[i,t - 1], 0, c1[1,t - 1], E1[t - 1], Tau1, w[3],asdf1, Sigma1)
					k2_C1 = h * nC(c1[i,t - 1] + k1_C1 * h/2, 0, c1[1,t - 1] + k1_C1 * h/2, E1[t - 1] + k1_E1 * h/2, Tau1, w[3],asdf1, Sigma1)
					k3_C1 = h * nC(c1[i,t - 1] + k2_C1 * h/2, 0, c1[1,t - 1] + k2_C1 * h/2, E1[t - 1] + k2_E1 * h/2, Tau1, w[3],asdf1, Sigma1)
					k4_C1 = h * nC(c1[i,t - 1] + k3_C1 * h, 0, c1[1,t - 1] + k3_C1 * h, E1[t - 1] + k3_E1 * h, Tau1, w[3],asdf1, Sigma1)

				if i == 1:
					k1_C1 = h * nC(c1[i,t - 1], 0, c1[i+1,t - 1], E2[t - 1], Tau1, w[3],asdf, Sigma1)
					k2_C1 = h * nC(c1[i,t - 1] + k1_C1 * h/2, 0, c1[i+1,t - 1] + k1_C1 * h/2, E2[t - 1] + k1_E2 * h/2, Tau1, w[3],asdf, Sigma1)
					k3_C1 = h * nC(c1[i,t - 1] + k2_C1 * h/2, 0, c1[i+1,t - 1] + k2_C1 * h/2, E2[t - 1] + k2_E2 * h/2, Tau1, w[3],asdf, Sigma1)
					k4_C1 = h * nC(c1[i,t - 1] + k3_C1 * h, 0, c1[i+1,t - 1] + k3_C1 * h, E2[t - 1] + k3_E2 * h, Tau1, w[3],asdf, Sigma1)

				if i == 4:
					k1_C1 = h * nC(c1[i,t - 1], 0, c1[i+1,t - 1], E1[t - 1], Tau1, w[3],asdf, Sigma1)
					k2_C1 = h * nC(c1[i,t - 1] + k1_C1 * h/2, 0, c1[i+1,t - 1] + k1_C1 * h/2, E1[t - 1] + k1_E1 * h/2, Tau1, w[3],asdf, Sigma1)
					k3_C1 = h * nC(c1[i,t - 1] + k2_C1 * h/2, 0, c1[i+1,t - 1] + k2_C1 * h/2, E1[t - 1] + k2_E1 * h/2, Tau1, w[3],asdf, Sigma1)
					k4_C1 = h * nC(c1[i,t - 1] + k3_C1 * h, 0, c1[i+1,t - 1] + k3_C1 * h, E1[t - 1] + k3_E1 * h, Tau1, w[3],asdf, Sigma1)

				if i == 3 or i == 5:
					k1_C1 = h * nC(c1[i,t - 1], 0, c1[4,t - 1], E2[t - 1], Tau1, w[4],asdf1, Sigma1)
					k2_C1 = h * nC(c1[i,t - 1] + k1_C1 * h/2, 0, c1[4,t - 1] + k1_C1 * h/2, E2[t - 1] + k1_E2 * h/2, Tau1, w[4],asdf1, Sigma1)
					k3_C1 = h * nC(c1[i,t - 1] + k2_C1 * h/2, 0, c1[4,t - 1] + k2_C1 * h/2, E2[t - 1] + k2_E2 * h/2, Tau1, w[4],asdf1, Sigma1)
					k4_C1 = h * nC(c1[i,t - 1] + k3_C1 * h, 0, c1[4,t - 1] + k3_C1 * h, E2[t - 1] + k3_E2 * h, Tau1, w[4],asdf1, Sigma1)

				c1[i,t] = c1[i,t - 1] + (1/6)*(k1_C1 + 2 * k2_C1 + 2 * k3_C1 + k4_C1)

				if i == 0 or i == 2:
					k1_C2 = h * nC(c2[i,t - 1], 0, c2[1,t - 1], E1[t - 1], Tau2, w[3],asdf1, Sigma2)
					k2_C2 = h * nC(c2[i,t - 1] + k1_C2 * h/2, 0, c2[1,t - 1] + k1_C2 * h/2, E1[t - 1] + k1_E1 * h/2, Tau2, w[3],asdf1, Sigma2)
					k3_C2 = h * nC(c2[i,t - 1] + k2_C2 * h/2, 0, c2[1,t - 1] + k2_C2 * h/2, E1[t - 1] + k2_E1 * h/2, Tau2, w[3],asdf1, Sigma2)
					k4_C2 = h * nC(c2[i,t - 1] + k3_C2 * h, 0, c2[1,t - 1] + k3_C2 * h, E1[t - 1] + k3_E1 * h, Tau2, w[3],asdf1, Sigma2)

				if i == 1:
					k1_C2 = h * nC(c2[i,t - 1], 0, c2[i+1,t - 1], E2[t - 1], Tau2, w[3],asdf, Sigma2)
					k2_C2 = h * nC(c2[i,t - 1] + k1_C2 * h/2, 0, c2[i+1,t - 1] + k1_C2 * h/2, E2[t - 1] + k1_E2 * h/2, Tau2, w[3],asdf, Sigma2)
					k3_C2 = h * nC(c2[i,t - 1] + k2_C2 * h/2, 0, c2[i+1,t - 1] + k2_C2 * h/2, E2[t - 1] + k2_E2 * h/2, Tau2, w[3],asdf, Sigma2)
					k4_C2 = h * nC(c2[i,t - 1] + k3_C2 * h, 0, c2[i+1,t - 1] + k3_C2 * h, E2[t - 1] + k3_E2 * h, Tau2, w[3],asdf, Sigma2)

				if i == 4:
					k1_C2 = h * nC(c2[i,t - 1], 0, c2[i+1,t - 1], E1[t - 1], Tau2, w[3],asdf, Sigma2)
					k2_C2 = h * nC(c2[i,t - 1] + k1_C2 * h/2, 0, c2[i+1,t - 1] + k1_C2 * h/2, E1[t - 1] + k1_E1 * h/2, Tau2, w[3],asdf, Sigma2)
					k3_C2 = h * nC(c2[i,t - 1] + k2_C2 * h/2, 0, c2[i+1,t - 1] + k2_C2 * h/2, E1[t - 1] + k2_E1 * h/2, Tau2, w[3],asdf, Sigma2)
					k4_C2 = h * nC(c2[i,t - 1] + k3_C2 * h, 0, c2[i+1,t - 1] + k3_C2 * h, E1[t - 1] + k3_E1 * h, Tau2, w[3],asdf, Sigma2)

				if i == 3 or i == 5:
					k1_C2 = h * nC(c2[i,t - 1], 0, c2[4,t - 1], E2[t - 1], Tau2, w[4],asdf1, Sigma2)
					k2_C2 = h * nC(c2[i,t - 1] + k1_C2 * h/2, 0, c2[4,t - 1] + k1_C2 * h/2, E2[t - 1] + k1_E2 * h/2, Tau2, w[4],asdf1, Sigma2)
					k3_C2 = h * nC(c2[i,t - 1] + k2_C2 * h/2, 0, c2[4,t - 1] + k2_C2 * h/2, E2[t - 1] + k2_E2 * h/2, Tau2, w[4],asdf1, Sigma2)
					k4_C2 = h * nC(c2[i,t - 1] + k3_C2 * h, 0, c2[4,t - 1] + k3_C2 * h, E2[t - 1] + k3_E2 * h, Tau2, w[4],asdf1, Sigma2)

				c2[i,t] = c2[i,t - 1] + (1/6)*(k1_C2 + 2 * k2_C2 + 2 * k3_C2 + k4_C2)

				z+=1
	return (c1, c2, E1, E2)

h = 1
stime = 1000
time = np.linspace(0, stime, int(stime/h))
c1, c2, E1, E2 = RK4(stime, h)


plt.figure(2)
plt.plot(time, c1[0,:])
plt.plot(time, c2[0,:])
# plt.plot(time, c[2,:])
# plt.plot(time, c[3,:])
# plt.plot(time, c[4,:])
# plt.plot(time, c[5,:])
plt.legend(['c0','c1','c2','c3','c4','c5'])

plt.show()