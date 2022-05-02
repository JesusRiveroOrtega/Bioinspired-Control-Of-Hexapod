from __future__ import division
import numpy as np
from numpy import genfromtxt
from scipy import signal
from matplotlib import pyplot as plt



datos = genfromtxt('/home/lutholum/datos.csv', delimiter=',')

samplePeriod = 1/50
#xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod)
#time = datos[:, 1-1]
#gyrX = datos[:, 2-1]
#gyrY = datos[:, 3-1]
#gyrZ = datos[:, 4-1]
accX = datos[1399:1700, 0]
#accY = datos[:, 6-1]
#accZ = datos[:, 7-1]
step = datos[1399:1700, 1]
#acc = [accX, accY]

# Detect stationary periods

# Compute accelerometer magnitude
acc_mag_x = np.sqrt(accX * accX)
#acc_mag_y = np.sqrt(accY * accY)

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

x = np.convolve(np.array(acc_magFilt_x), b[0][0])
x = x + np.flip(np.convolve(np.array(np.flip(acc_magFilt_x)), b[0][0]))
#0.082
stationary_x = acc_magFilt_x < 2

plt.subplot(2,1,1)
plt.plot(stationary_x, lineWidth=2)
plt.subplot(2,1,2)
plt.plot(acc_magFilt_x, lineWidth=2)
plt.show()


stationaryStart_x = np.nonzero(np.concatenate((np.zeros((1)), np.diff(stationary_x.astype(int))), axis=0) == -1)
stationaryEnd_x = np.nonzero(np.concatenate((np.zeros((1)), np.diff(stationary_x.astype(int))), axis = 0) == 1)
stationary_x = stationary_x.astype(int)
stationaryStart_x = stationaryStart_x[0].astype(int)
stationaryEnd_x = stationaryEnd_x[0].astype(int)
#idx = np.zeros((1))
#for i in range(0, 2):
#    rg = np.arange(stationaryStart_x[i], stationaryEnd_x[i])
#    idx = np.concatenate((idx, rg), axis=0)
#idx = idx.astype(int)
#stationary_x[np.array(idx)] = stationary_x[np.array(idx)]*0
stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]] = stationary_x[stationaryStart_x[0]:stationaryEnd_x[len(stationaryEnd_x)-1]]*0



# Integrate acceleration to yield velocity
vel_x = np.zeros(np.shape(accX))
for t in range(1, len(vel_x)):
    vel_x[t] = vel_x[t-1] + accX[t] * step[t]
    if(stationary_x[t] == 1):
        vel_x[t] = 0    # force zero velocity when foot stationary


# Compute integral drift during non-stationary periods
velDrift_x = np.zeros((np.shape(vel_x)))

for i in range(0, len(stationaryEnd_x)):
    driftRate_x = vel_x[stationaryEnd_x[i]-1] / (stationaryEnd_x[i] - stationaryStart_x[i])
    ss = (stationaryEnd_x[i] - stationaryStart_x[i])
    enum_x = np.arange(0, ss)

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

plt.plot(pos, lineWidth=2)
plt.show()
# Plot translational position
print(pos[len(pos)-1])
