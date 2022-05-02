#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import sys, select, termios, tty
import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hexapodo.msg import Num
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64

tau_current = 1.5
tau_setpoint = 1.5
tau_cur_m_set = 1.5
tau_set_m_cur = 1.5
tau_left_tank = 0.2
tau_right_tank = 0.2

u = 12

input_angle = 0
current_angle = 0

angle_step = int(360/u)

preferred_direction = np.linspace(-np.pi, np.pi, u, endpoint = False)

h = 0.1

# synapses11 = 9*np.ones((5))
# synapses12 = 9*np.ones((5))
# synapses21 = 9*np.ones((5))
# synapses22 = 9*np.ones((5))

synapses11 = 1*np.ones((u))
synapses12 = 1*np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1])
synapses21 = 1*np.ones((u))
synapses22 = 1*np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1])

prev_tank = 1
prev_tank_1 = 0
tanks_syn = 0
#synapses1 =  * np.ones((11))



def circ_conv(fltr, inputs):
    n = len(fltr)
    conv = np.convolve(fltr, np.hstack((inputs, inputs, inputs)))

    conv = conv[int(np.fix(n / 2) + len(inputs)): int(np.fix(n / 2) + 2 * len(inputs))]
    return conv

def Neuron(Neuron, Input, M, S, A, tau):
	return (1/tau)*(- Neuron + NR(Input, M, S, A))


def NR(X, M, S, A):
	return M * ((X * (X > 0)) ** 2)/ ((S + A )** 2 + ((X * (X > 0)) ** 2))


def Setpoint(data):
    global input_angle
    input_angle = data.data

def Current(data):
    global current_angle
    current_angle = data.data

def callback(imu_data):
    global current_angle

    orientation = imu_data.orientation
    orientation_l = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_l)
    yaw = np.rad2deg(yaw)
    current_angle = (yaw - 360 * np.fix(yaw / 360))*(yaw > 0) + (yaw + 360) * (yaw < 0)

def RK4():
    global current_angle

    rospy.init_node('rotational_control')
    rospy.Subscriber("/angle_setpoint", Float64, Setpoint)
    # rospy.Subscriber("/angle_current", Float64, Current)
    rotation_direction = rospy.Publisher('/rotation_direction', Int64, queue_size = 1)
    dir_mov = rospy.Publisher('/rotation_direction_scale', Float64, queue_size = 1)
    L_R_scaling = rospy.Publisher('/L_R_scaling', Num, queue_size = 1)
    rospy.Subscriber("/imu", Imu, callback)
    n = 2

    rate = rospy.Rate(100) # 10hz

    distance = 100



    setpoint = np.zeros((u, n), dtype = np.float64)
    current_status = np.zeros((u, n), dtype = np.float64)
    cur_m_set = np.zeros((u, n), dtype = np.float64)
    set_m_cur = np.zeros((u, n), dtype = np.float64)
    left_tank = np.zeros((1, n), dtype = np.float64)
    right_tank = np.zeros((1, n), dtype = np.float64)

    while not rospy.is_shutdown():

        input_setpoint = distance*np.cos(preferred_direction - np.radians(input_angle))*(distance*np.cos(preferred_direction - np.radians(input_angle)) >= 0)
        input_current = distance*np.cos(preferred_direction - np.radians(current_angle))*(distance*np.cos(preferred_direction - np.radians(current_angle)) >=0)


        k1_setpoint =       h * Neuron(setpoint[:, 0], input_setpoint, 100, 25, 0, tau_setpoint)
        k1_current_status = h * Neuron(current_status[:, 0], input_current, 100, 25, 0, tau_current)

        k1_cur_m_set =      h * Neuron(cur_m_set[:, 0], synapses11*current_status[:, 0] - circ_conv(synapses12,setpoint[:, 0]), 100, 50, 0, tau_cur_m_set)
        k1_set_m_cur =      h * Neuron(set_m_cur[:, 0], synapses21*setpoint[:, 0] - circ_conv(synapses22,current_status[:, 0]), 100, 50, 0, tau_set_m_cur)

        k1_left_tank =      h * Neuron(left_tank[:, 0], prev_tank*np.sum(cur_m_set[:, 0]) - prev_tank_1*np.sum(set_m_cur[:, 0]) - tanks_syn*right_tank[:, 0], 100, 50, 0, tau_left_tank)
        k1_right_tank =     h * Neuron(right_tank[:, 0], prev_tank*np.sum(set_m_cur[:, 0]) - prev_tank_1*np.sum(cur_m_set[:, 0]) - tanks_syn*left_tank[:, 0], 100, 50, 0, tau_right_tank)

        k2_setpoint =       h * Neuron(setpoint[:, 0] + k1_setpoint*h/2, input_setpoint, 100, 25, 0, tau_setpoint)
        k2_current_status = h * Neuron(current_status[:, 0] + k1_current_status*h/2, input_current, 100, 25, 0, tau_current)

        k2_cur_m_set =      h * Neuron(cur_m_set[:, 0] + k1_cur_m_set*h/2, synapses11*(current_status[:, 0] + k1_current_status*h/2) - circ_conv(synapses12,(setpoint[:, 0] + k1_setpoint*h/2)), 100, 50, 0, tau_cur_m_set)
        k2_set_m_cur =      h * Neuron(set_m_cur[:, 0] + k1_set_m_cur*h/2, synapses21*(setpoint[:, 0] + k1_setpoint*h/2) - circ_conv(synapses22,(current_status[:, 0] + k1_current_status*h/2)), 100, 50, 0, tau_set_m_cur)

        k2_left_tank =      h * Neuron(left_tank[:, 0] + k1_left_tank*h/2, prev_tank*np.sum(cur_m_set[:, 0] + k1_cur_m_set*h/2) - prev_tank_1*np.sum(set_m_cur[:, 0] + k1_set_m_cur*h/2) - tanks_syn*(right_tank[:, 0] + k1_right_tank*h/2), 100, 50, 0, tau_left_tank)
        k2_right_tank =     h * Neuron(right_tank[:, 0] + k1_right_tank*h/2, prev_tank*np.sum(set_m_cur[:, 0] + k1_set_m_cur*h/2) - prev_tank_1*np.sum(cur_m_set[:, 0] + k1_cur_m_set*h/2) - tanks_syn*(left_tank[:, 0] + k1_left_tank*h/2), 100, 50, 0, tau_right_tank)


        k3_setpoint =       h * Neuron(setpoint[:, 0] + k2_setpoint*h/2, input_setpoint, 100, 25, 0, tau_setpoint)
        k3_current_status = h * Neuron(current_status[:, 0] + k2_current_status*h/2, input_current, 100, 25, 0, tau_current)

        k3_cur_m_set =      h * Neuron(cur_m_set[:, 0] + k2_cur_m_set*h/2, synapses11*(current_status[:, 0] + k2_current_status*h/2) - circ_conv(synapses12,(setpoint[:, 0] + k2_setpoint*h/2)), 100, 50, 0, tau_cur_m_set)
        k3_set_m_cur =      h * Neuron(set_m_cur[:, 0] + k2_set_m_cur*h/2, synapses21*(setpoint[:, 0] + k2_setpoint*h/2) - circ_conv(synapses22,(current_status[:, 0] + k2_current_status*h/2)), 100, 50, 0, tau_set_m_cur)

        k3_left_tank =      h * Neuron(left_tank[:, 0] + k2_left_tank*h/2, prev_tank*np.sum(cur_m_set[:, 0] + k2_cur_m_set*h/2) - prev_tank_1*np.sum(set_m_cur[:, 0] + k2_set_m_cur*h/2) - tanks_syn*(right_tank[:, 0] + k2_right_tank*h/2), 100, 50, 0, tau_left_tank)
        k3_right_tank =     h * Neuron(right_tank[:, 0] + k2_right_tank*h/2, prev_tank*np.sum(set_m_cur[:, 0] + k2_set_m_cur*h/2) - prev_tank_1*np.sum(cur_m_set[:, 0] + k2_cur_m_set*h/2) - tanks_syn*(left_tank[:, 0] + k2_left_tank*h/2), 100, 50, 0, tau_right_tank)


        k4_setpoint =       h * Neuron(setpoint[:, 0] + k3_setpoint*h, input_setpoint, 100, 25, 0, tau_setpoint)
        k4_current_status = h * Neuron(current_status[:, 0] + k3_current_status*h, input_current, 100, 25, 0, tau_current)

        k4_cur_m_set =      h * Neuron(cur_m_set[:, 0] + k3_cur_m_set*h, synapses11*(current_status[:, 0] + k3_current_status*h) - circ_conv(synapses12,(setpoint[:, 0] + k3_setpoint*h)), 100, 50, 0, tau_cur_m_set)
        k4_set_m_cur =      h * Neuron(set_m_cur[:, 0] + k3_set_m_cur*h, synapses21*(setpoint[:, 0] + k3_setpoint*h) - circ_conv(synapses22,(current_status[:, 0] + k3_current_status*h)), 100, 50, 0, tau_set_m_cur)

        k4_left_tank =      h * Neuron(left_tank[:, 0] + k3_left_tank*h, prev_tank*np.sum(cur_m_set[:, 0] + k3_cur_m_set*h) - prev_tank_1*np.sum(set_m_cur[:, 0] + k3_set_m_cur*h) - tanks_syn*(right_tank[:, 0] + k1_right_tank*h), 100, 50, 0, tau_left_tank)
        k4_right_tank =     h * Neuron(right_tank[:, 0] + k3_right_tank*h, prev_tank*np.sum(set_m_cur[:, 0] + k3_set_m_cur*h) - prev_tank_1*np.sum(cur_m_set[:, 0] + k3_cur_m_set*h) - tanks_syn*(left_tank[:, 0] + k1_left_tank*h), 100, 50, 0, tau_right_tank)



        setpoint[:, 1]       = setpoint[:, 0]       + (1/6)*(k1_setpoint       + 2 * k2_setpoint       + 2 * k3_setpoint       + k4_setpoint)
        current_status[:, 1] = current_status[:, 0] + (1/6)*(k1_current_status + 2 * k2_current_status + 2 * k3_current_status + k4_current_status)
        cur_m_set[:, 1]      = cur_m_set[:, 0]      + (1/6)*(k1_cur_m_set      + 2 * k2_cur_m_set      + 2 * k3_cur_m_set      + k4_cur_m_set)
        set_m_cur[:, 1]      = set_m_cur[:, 0]      + (1/6)*(k1_set_m_cur      + 2 * k2_set_m_cur      + 2 * k3_set_m_cur      + k4_set_m_cur)
        left_tank[:, 1]      = left_tank[:, 0]      + (1/6)*(k1_left_tank      + 2 * k2_left_tank      + 2 * k3_left_tank      + k4_left_tank)
        right_tank[:, 1]     = right_tank[:, 0]     + (1/6)*(k1_right_tank     + 2 * k2_right_tank     + 2 * k3_right_tank     + k4_right_tank)



        setpoint[:, 0]       = setpoint[:, 1]
        current_status[:, 0] = current_status[:, 1]
        cur_m_set[:, 0]      = cur_m_set[:, 1]
        set_m_cur[:, 0]      = set_m_cur[:, 1]
        left_tank[:, 0]      = left_tank[:, 1]
        right_tank[:, 0]     = right_tank[:, 1]



        direct_movement = (1/1)*(left_tank[:, 1]) - (1/1)*(right_tank[:, 1])
        
        dir_mov.publish(direct_movement)
        print(" -- - -- - -- - -- - -- - -- - --")
        print([input_angle, current_angle])
        print(direct_movement)
        print(" -- - -- - -- - -- - -- - -- - --")

        if direct_movement > 0.15:
            rotation_direction.publish(1)
        elif direct_movement < -0.15:
            rotation_direction.publish(-1)
        elif direct_movement <= 0.15 and direct_movement >= -0.15:
            rotation_direction.publish(0)


        Lst = 70.67
        Rst = 70.67
        Lu = 95.58
        Ru = 95.47

        Lo = (0.9/(Lst - Lu))*left_tank[:, 1] + (1 - (0.9/(Lst - Lu))*Lst)
        Ro = (0.9/(Rst - Ru))*right_tank[:, 1] + (1 - (0.9/(Rst - Ru))*Rst)
        L_R_scaling.publish(np.array([Lo, Ro]))

        rate.sleep()

        # print([input_angle, current_angle])






if __name__ == '__main__':
	try:
		RK4()
	except rospy.ROSInterruptException:
		pass
