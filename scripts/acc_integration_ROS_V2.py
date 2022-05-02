#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import numpy as np
from scipy import signal
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from hexapodo.msg import Num

distance_msg = []
enabler = 0
orientation = 0

def get_distance(data):
    global distance_msg, enabler, orientation
    datos = data.num
    orientation = datos[0]
    datos = np.reshape(datos[1:], (-1, 5))

    samplePeriod = 1/50
    time = datos[:, 0]
    accX = datos[:, 1]
    accY = datos[:, 2]
    accZ = datos[:, 3]
    step = datos[:, 4]

    acc_mag_x = np.sqrt(accX * accX)
    print(acc_mag_x)
    filtCutOff_x = 0.0001
    [b_x, a_x] = signal.butter(1, (2*filtCutOff_x)/(1/samplePeriod), 'high')

    acc_magFilt_x = signal.filtfilt(b_x, a_x, acc_mag_x, axis=0)

    acc_magFilt_x = np.abs(acc_magFilt_x)

    filtCutOff_x = 5
    [b_x, a_x] = signal.butter(1, (2*filtCutOff_x)/(1/samplePeriod), 'low')
    acc_magFilt_x = signal.filtfilt(b_x, a_x, acc_magFilt_x, axis=0)



    stationary_x = acc_magFilt_x < 2

    stationaryStart_x = np.nonzero(np.concatenate((np.zeros((1)), np.diff(stationary_x.astype(int))), axis=0) == -1)
    stationaryEnd_x = np.nonzero(np.concatenate((np.zeros((1)), np.diff(stationary_x.astype(int))), axis = 0) == 1)
    stationary_x = stationary_x.astype(int)
    stationaryStart_x = stationaryStart_x[0].astype(int)
    stationaryEnd_x = stationaryEnd_x[0].astype(int)

    if np.shape(stationaryStart_x) != (0,):
        stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]] = stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]]*0



    vel_x = np.zeros(np.shape(accX))
    for t in range(1, len(vel_x)):
        vel_x[t] = vel_x[t-1] + accX[t] * step[t]
        if(stationary_x[t] == 1):
            vel_x[t] = 0


    velDrift_x = np.zeros((np.shape(vel_x)))

    if np.shape(stationaryStart_x) != (0,):

        for i in range(0, len(stationaryEnd_x)):
            driftRate_x = vel_x[stationaryEnd_x[i]-1] / (stationaryEnd_x[i] - stationaryStart_x[i])
            ss = (stationaryEnd_x[i] - stationaryStart_x[i])
            enum_x = np.arange(0, ss)

            drift_x = enum_x*driftRate_x
            velDrift_x[stationaryStart_x[i]:stationaryEnd_x[i]] = drift_x


    vel_x = vel_x - velDrift_x

    vel = vel_x

    pos = np.zeros((np.shape(vel)))
    for t in range(1, len(pos)):
        pos[t] = pos[t-1] + vel[t] * step[t]



    distance_msg = [pos[len(pos) - 1]]
    print(distance_msg)
    enabler = 1

def acc_integ():

    global distance_msg, enabler, orientation
    rospy.init_node('acc_integration')
    rospy.Subscriber("/acc_raw", Num, get_distance)
    acc_raw = rospy.Publisher('/distance', Num, queue_size=1)
    orientation_msg = rospy.Publisher('/orientation', Num, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():

        if enabler == 1:
            orientation_msg.publish(np.array([orientation]))
            #acc_raw.publish(np.array(distance_msg))
            acc_raw.publish(np.array([1.6]))
            enabler = 0
        rate.sleep()

if __name__ == '__main__':
	try:
		acc_integ()
	except rospy.ROSInterruptException:
		pass
