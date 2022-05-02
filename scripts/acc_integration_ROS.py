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

def get_distance(data):
    global distance_msg, enabler
    datos = data.num
    print(datos)
    datos = np.reshape(datos, (-1, 5))

    samplePeriod = 1/50
    #xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod)
    time = datos[:, 0]
    accX = datos[:, 1]
    accY = datos[:, 2]
    accZ = datos[:, 3]
    step = datos[:, 4]


    # Detect stationary periods


    # Compute accelerometer magnitude
    acc_mag_x = np.sqrt(accX * accX)


    # HP filter accelerometer data  2
    filtCutOff_x = 0.0001
    [b_x, a_x] = signal.butter(1, (2*filtCutOff_x)/(1/samplePeriod), 'high')
    acc_magFilt_x = signal.filtfilt(b_x, a_x, acc_mag_x)

    # Compute absolute value
    acc_magFilt_x = np.abs(acc_magFilt_x)

    # LP filter accelerometer data  4.478  3
    filtCutOff_x = 5
    [b_x, a_x] = signal.butter(1, (2*filtCutOff_x)/(1/samplePeriod), 'low')
    acc_magFilt_x = signal.filtfilt(b_x, a_x, acc_magFilt_x)

    windowSize = 19
    b = np.array([(1/windowSize)*np.ones((1,windowSize))])
    a = np.array([1])

    x = np.convolve(np.array(acc_magFilt_x), b[0][0], 'same')
    #0.082
    stationary_x = acc_magFilt_x < 2
    stationary_x = stationary_x.astype(int)


    prel = np.concatenate((np.zeros((1)), np.diff(stationary_x)), axis = 0)
    prel = prel.astype(int)

    stationaryStart_x = np.nonzero(prel == -1)
    stationaryEnd_x   = np.nonzero(prel == 1)    

    if np.shape(stationaryStart_x)[0] < np.shape(stationaryEnd_x)[0]:
        stationaryStart_x = np.concatenate((np.zeros((1)), stationaryStart_x), axis = 0)

    if np.shape(stationaryStart_x)[0] > np.shape(stationaryEnd_x)[0]:
        stationaryEnd_x = np.concatenate((stationaryEnd_x, np.shape(stationaryStart_x)[0] - 1), axis = 0)


    stationaryStart_x = stationaryStart_x[0].astype(int)
    stationaryEnd_x = stationaryEnd_x[0].astype(int)


    idx = np.zeros((1))
    for i in range(0, np.shape(stationaryStart_x)[0] - 1):
        rg = np.arange(stationaryStart_x[i], stationaryEnd_x[i])
        idx = np.concatenate((idx, rg), axis=0)
    idx = idx.astype(int)
    stationary_x[np.array(idx)] = stationary_x[np.array(idx)]*0
    if np.shape(stationaryStart_x)[0] != 0:
        stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]] = stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]]*0


    # Integrate acceleration to yield velocity
    vel_x = np.zeros(np.shape(accX))
    for t in range(1, len(vel_x)):
        vel_x[t] = vel_x[t-1] + accX[t] * step[t]
        if(stationary_x[t] == 1):
            vel_x[t] = 0    # force zero velocity when foot stationary


    # Compute integral drift during non-stationary periods
    velDrift_x = np.zeros((np.shape(vel_x)))

    for i in range(0, len(stationaryEnd_x)-1):
        driftRate_x = vel_x[stationaryEnd_x[i]-1] / (stationaryEnd_x[i] - stationaryStart_x[i])
        enum_x = np.arange(0, (stationaryEnd_x[i] - stationaryStart_x[i]))

        drift_x = enum_x*driftRate_x
        velDrift_x[stationaryStart_x[i]:stationaryEnd_x[i]] = drift_x


    # Remove integral drift
    vel_x = vel_x - velDrift_x



    #}
    vel = vel_x

    # -------------------------------------------------------------------------
    # Compute translational position


    # Integrate velocity to yield position
    pos = np.zeros((np.shape(vel)))
    for t in range(1, len(pos)):
        pos[t] = pos[t-1] + vel[t] * step[t]    # integrate velocity to yield position


    # Plot translational position
    #acc_raw.publish(np.array([pos[len(pos)-1]]))
    distance_msg = [pos[len(pos) - 1]]
    enabler = 1
    #print(distance_msg)


def acc_integ():

    global distance_msg, enabler
    rospy.init_node('acc_integration')
    rospy.Subscriber("/acc_raw", Num, get_distance)
    acc_raw = rospy.Publisher('/distance', Num, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():

        if enabler == 1:
            acc_raw.publish(np.array(distance_msg))
            enabler = 0
        rate.sleep()

	#rospy.spin()

if __name__ == '__main__':
	try:
		acc_integ()
	except rospy.ROSInterruptException:
		pass
