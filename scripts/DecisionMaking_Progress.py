
import numpy as np
from matplotlib import pyplot as plt
from main2 import *

# print('''
# 	Virtual Hexapod Project
# 	Scalable Oponent Control
# 	Basal Ganglia Application''')

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
	return (1/0.2)*(-neu + naka_rushton(5, 3, 2, x_1 + 2*neu))

def x_7(neu, x_2):
	return (1/0.2)*(-neu + naka_rushton(5, 4, 2, x_2 + 2*neu))





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

def RK4(time, h):


	vn = int(time/h)


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

	print([m_r, m_b, m_i])
	print([b_r, b_b, b_i])
	# print(w_b, w_r)
	L = 0
	X_1 = np.zeros((vn))
	X_2 = np.zeros((vn))
	X_3 = np.zeros((vn))
	X_4 = np.zeros((vn))
	X_5 = np.zeros((vn))
	X_6 = np.zeros((vn))
	X_7 = np.zeros((vn))

	t = 0
	L_c = []
	S_1_c = []
	S_2_c = []

	while t != vn:

		#D_b = 0.5*489/w_b
		#D_r = 0.5*489/w_r

		D_b =10*(t < 500/0.02) + 3*(t >= 500/0.02 and t < 700/0.02) + 100*(t >= 700/0.02)
		D_r =10*(t < 500/0.02) + 20*(t >= 500/0.02 and t < 700/0.02) + 100*(t >= 700/0.02)

		S_1 = (D_r < 4.2)*(m_r*D_r + b_r) + (D_r >= 4.2 and D_r < 8)*(m_i*D_r + b_i+0.02) + (D_r >= 8)*(0.11) #Predator signal
		S_2 = (D_b < 4.2)*(m_b*D_b + b_b) + (D_b >= 4.2 and D_b < 8)*(m_i*D_b + b_i) + (D_b >= 8)*(0.11) #Prey signal


		s1 = S_1 + 0.05 * S_1 *np.random.normal(0,1)*0
		s2 = S_2 + 0.05 * S_2 *np.random.normal(0,1)*0
		L_m = L + 0.05 * L *np.random.normal(0,1)*0
		S_1_c.append(S_1)
		S_2_c.append(S_2)
		L_c.append(L_m)


		k1_X_1 = h * x_1(X_1[t - 1], s1*(L == 0), X_2[t - 1], L_m)
		k1_X_2 = h * x_2(X_2[t - 1], s2*(L == 0), X_1[t - 1], L_m)
		k1_X_3 = h * x_3(X_3[t - 1], X_1[t - 1], X_2[t - 1])
		k1_X_4 = h * x_4(X_4[t - 1], X_1[t - 1], X_2[t - 1])
		k1_X_5 = h * x_5(X_5[t - 1], X_3[t - 1], X_4[t - 1], X_6[t - 1], X_7[t - 1])
		k1_X_6 = h * x_6(X_6[t - 1], X_1[t - 1])
		k1_X_7 = h * x_7(X_7[t - 1], X_2[t - 1])


		k2_X_1 = h * x_1(X_1[t - 1] + k1_X_1 * h/2, s1*(L == 0), X_2[t - 1] + k1_X_2 * h/2, L_m)
		k2_X_2 = h * x_2(X_2[t - 1] + k1_X_2 * h/2, s2*(L == 0), X_1[t - 1] + k1_X_1 * h/2, L_m)
		k2_X_3 = h * x_3(X_3[t - 1] + k1_X_3 * h/2, X_1[t - 1] + k1_X_1 * h/2, X_2[t - 1] + k1_X_2 * h/2)
		k2_X_4 = h * x_4(X_4[t - 1] + k1_X_4 * h/2, X_1[t - 1] + k1_X_1 * h/2, X_2[t - 1] + k1_X_2 * h/2)
		k2_X_5 = h * x_5(X_5[t - 1] + k1_X_5 * h/2, X_3[t - 1] + k1_X_3 * h/2, X_4[t - 1] + k1_X_4 * h/2, X_6[t - 1] + k1_X_6 * h/2, X_7[t - 1] + k1_X_7 * h/2)
		k2_X_6 = h * x_6(X_6[t - 1] + k1_X_6 * h/2, X_1[t - 1] + k1_X_1 * h/2)
		k2_X_7 = h * x_7(X_7[t - 1] + k1_X_7 * h/2, X_2[t - 1] + k1_X_2 * h/2)


		k3_X_1 = h * x_1(X_1[t - 1] + k2_X_1 * h/2, s1*(L == 0), X_2[t - 1] + k2_X_2 * h/2, L_m)
		k3_X_2 = h * x_2(X_2[t - 1] + k2_X_2 * h/2, s2*(L == 0), X_1[t - 1] + k2_X_1 * h/2, L_m)
		k3_X_3 = h * x_3(X_3[t - 1] + k2_X_3 * h/2, X_1[t - 1] + k2_X_1 * h/2, X_2[t - 1] + k2_X_2 * h/2)
		k3_X_4 = h * x_4(X_4[t - 1] + k2_X_4 * h/2, X_1[t - 1] + k2_X_1 * h/2, X_2[t - 1] + k2_X_2 * h/2)
		k3_X_5 = h * x_5(X_5[t - 1] + k2_X_5 * h/2, X_3[t - 1] + k2_X_3 * h/2, X_4[t - 1] + k2_X_4 * h/2, X_6[t - 1] + k2_X_6 * h/2, X_7[t - 1] + k2_X_7 * h/2)
		k3_X_6 = h * x_6(X_6[t - 1] + k2_X_6 * h/2, X_1[t - 1] + k2_X_1 * h/2)
		k3_X_7 = h * x_7(X_7[t - 1] + k2_X_7 * h/2, X_2[t - 1] + k2_X_2 * h/2)

		k4_X_1 = h * x_1(X_1[t - 1] + k3_X_1 * h, s1*(L == 0), X_2[t - 1] + k3_X_2 * h, L_m)
		k4_X_2 = h * x_2(X_2[t - 1] + k3_X_2 * h, s2*(L == 0), X_1[t - 1] + k3_X_1 * h, L_m)
		k4_X_3 = h * x_3(X_3[t - 1] + k3_X_3 * h, X_1[t - 1] + k3_X_1 * h, X_2[t - 1] + k3_X_2 * h)
		k4_X_4 = h * x_4(X_4[t - 1] + k3_X_4 * h, X_1[t - 1] + k3_X_1 * h, X_2[t - 1] + k3_X_2 * h)
		k4_X_5 = h * x_5(X_5[t - 1] + k3_X_5 * h, X_3[t - 1] + k3_X_3 * h, X_4[t - 1] + k3_X_4 * h, X_6[t - 1] + k3_X_6 * h, X_7[t - 1] + k3_X_7 * h)
		k4_X_6 = h * x_6(X_6[t - 1] + k3_X_6 * h, X_1[t - 1] + k3_X_1 * h)
		k4_X_7 = h * x_7(X_7[t - 1] + k3_X_7 * h, X_2[t - 1] + k3_X_2 * h)



		X_1[t] = X_1[t - 1] + (1/6)*(k1_X_1 + 2*k2_X_1 + 2*k3_X_1 + k4_X_1)
		X_2[t] = X_2[t - 1] + (1/6)*(k1_X_2 + 2*k2_X_2 + 2*k3_X_2 + k4_X_2)
		X_3[t] = X_3[t - 1] + (1/6)*(k1_X_3 + 2*k2_X_3 + 2*k3_X_3 + k4_X_3)
		X_4[t] = X_4[t - 1] + (1/6)*(k1_X_4 + 2*k2_X_4 + 2*k3_X_4 + k4_X_4)
		X_5[t] = X_5[t - 1] + (1/6)*(k1_X_5 + 2*k2_X_5 + 2*k3_X_5 + k4_X_5)
		X_6[t] = X_6[t - 1] + (1/6)*(k1_X_6 + 2*k2_X_6 + 2*k3_X_6 + k4_X_6)
		X_7[t] = X_7[t - 1] + (1/6)*(k1_X_7 + 2*k2_X_7 + 2*k3_X_7 + k4_X_7)


		t = t + 1
	return (X_1, X_2, X_3, X_4, X_5, X_6, X_7, S_1_c, S_2_c, L_c)


h = 0.02
stime = 1000
time = np.linspace(0, stime, int(stime/h))
Gl_M, Gl_A, Mo, Ao, Ci, Mi, Ai, S1_s, S2_s, L_s = RK4(stime, h)




plt.subplot('311')
plt.plot(time, S1_s, 'r',time, S2_s, 'b')
plt.xlabel('Tiempo (ms)')
plt.ylabel('Nivel de estimulación')
plt.legend(['S1', 'S2'])

plt.subplot('312')
plt.plot(time, Gl_M, 'r',time, Gl_A, 'b')
plt.xlabel('Tiempo (ms)')
plt.ylabel('Frecuencia de disparo (Hz)')
plt.legend(['X1', 'X2'])
plt.subplot('313')
plt.plot(time, Ci, 'c',time, Mi, 'g',time, Ai, 'm')
plt.xlabel('Tiempo (ms)')
plt.ylabel('Frecuencia de disparo (Hz)')
plt.legend(['X5', 'X6', 'X7'])
plt.show()
